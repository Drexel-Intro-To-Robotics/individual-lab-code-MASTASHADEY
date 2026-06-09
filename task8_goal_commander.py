#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Import the native ROS Inverse Kinematics Service
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

class OpenManipulatorTopicCommander:
    def __init__(self):
        rospy.init_node('task8_direct_commander', anonymous=True)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # 1. Publish directly to the raw command topic (Bypasses actionlib entirely)
        self.traj_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

        # 2. Connect to the native ROS Inverse Kinematics Service (Bypasses moveit_commander)
        rospy.loginfo("Waiting for /compute_ik service...")
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        rospy.loginfo("Connected to IK service!")

        # 3. Subscribers
        rospy.Subscriber('/goal_joint_space', Float64MultiArray, self.joint_space_callback)
        rospy.Subscriber('/goal_task_space', Point, self.task_space_callback)
        rospy.Subscriber('/goal_waypoints', geometry_msgs.msg.PoseArray, self.waypoints_callback)

        rospy.loginfo("Task 8 Direct Topic Commander Operational. Awaiting goals...")

    def compute_inverse_kinematics(self, x, y, z):
        """Uses the native ROS MoveIt service to calculate IK."""
        req = GetPositionIKRequest()
        req.ik_request.group_name = "arm"
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0]
        
        req.ik_request.pose_stamped.header.frame_id = "link1" 
        req.ik_request.pose_stamped.header.stamp = rospy.Time.now()
        
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        req.ik_request.pose_stamped.pose.orientation.w = 1.0 
        
        req.ik_request.timeout = rospy.Duration(1.0)
        
        try:
            resp = self.ik_service(req)
            if resp.error_code.val == 1:
                joints = []
                for name in self.joint_names:
                    idx = resp.solution.joint_state.name.index(name)
                    joints.append(resp.solution.joint_state.position[idx])
                return joints
            else:
                rospy.logwarn("IK Service failed. Target point out of reach.")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"IK Service call failed: {e}")
            return None

    def joint_space_callback(self, msg):
        """Executes a direct list of 4 joint angles."""
        if len(msg.data) != 4:
            rospy.logwarn("Joint space goal requires exactly 4 angles!")
            return
        rospy.loginfo(f"Executing Joint-Space Goal: {msg.data}")
        self.send_trajectory([msg.data], duration=3.0)

    def task_space_callback(self, msg):
        """Takes an X, Y, Z coordinate, runs the IK service, and executes."""
        rospy.loginfo(f"Executing Task-Space Goal: X={msg.x}, Y={msg.y}, Z={msg.z}")
        target_joints = self.compute_inverse_kinematics(msg.x, msg.y, msg.z)
        if target_joints:
            self.send_trajectory([target_joints], duration=3.0)

    def waypoints_callback(self, msg):
        """Accepts an array of Poses, computes IK for each, and executes a continuous path."""
        rospy.loginfo(f"Received {len(msg.poses)} waypoints. Computing IK path...")
        trajectory_points = []
        for pose in msg.poses:
            joints = self.compute_inverse_kinematics(pose.position.x, pose.position.y, pose.position.z)
            if joints:
                trajectory_points.append(joints)
            else:
                rospy.logerr("Failed to calculate IK for a waypoint. Aborting.")
                return
        self.send_trajectory(trajectory_points, duration=2.0)

    def send_trajectory(self, list_of_joint_arrays, duration=2.0):
        """
        Publishes a direct JointTrajectory message to the controller command topic.
         Bypasses actionlib and uses a 0 timestamp for instantaneous processing.
        """
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        # Setting stamp to 0 forces the robot to execute immediately 
        # and eliminates time synchronization crashes
        traj.header.stamp = rospy.Time(0)
        
        time_from_start = 0.0
        for joints in list_of_joint_arrays:
            point = JointTrajectoryPoint()
            point.positions = joints
            
            # Leave velocities/accelerations empty so ros_control calculates them smoothly
            point.velocities = []
            point.accelerations = []
            
            time_from_start += duration
            point.time_from_start = rospy.Duration(time_from_start)
            traj.points.append(point)

        # Publish the message directly to the topic
        self.traj_pub.publish(traj)
        rospy.loginfo("Trajectory published to /arm_controller/command.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        commander = OpenManipulatorTopicCommander()
        commander.run()
    except rospy.ROSInterruptException:
        pass
