#!/usr/bin/env python3
import rospy
import actionlib
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Point
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class OpenManipulatorDirectCommander:
    def __init__(self):
        rospy.init_node('task8_direct_commander', anonymous=True)

        # The OpenManipulator-X default joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Connect directly to the ros_control action server instead of MoveIt!
        self.client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for /arm_controller/follow_joint_trajectory action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to trajectory action server!")

        # Subscribers for the three types of goals
        rospy.Subscriber('/goal_joint_space', Float64MultiArray, self.joint_space_callback)
        rospy.Subscriber('/goal_task_space', Point, self.task_space_callback)
        rospy.Subscriber('/goal_waypoints', geometry_msgs.msg.PoseArray, self.waypoints_callback)

        rospy.loginfo("Task 8 Direct Goal Commander Operational. Awaiting goals...")

    def compute_inverse_kinematics(self, x, y, z):
        """
        TODO: Insert your Task 2 IK math here!
        Takes an X, Y, Z position and returns a list of 4 joint angles [j1, j2, j3, j4].
        """
        # --- YOUR MATH GOES HERE ---
        # Example dummy values:
        j1, j2, j3, j4 = 0.0, 0.0, 0.0, 0.0 
        
        return [j1, j2, j3, j4]

    def joint_space_callback(self, msg):
        """Accepts an array of 4 joint angles in radians and executes."""
        if len(msg.data) != 4:
            rospy.logwarn("Joint space goal requires exactly 4 angles!")
            return
        
        rospy.loginfo(f"Executing Joint-Space Goal: {msg.data}")
        self.send_trajectory([msg.data], duration=3.0)

    def task_space_callback(self, msg):
        """Accepts an X, Y, Z coordinate, runs IK, and executes."""
        rospy.loginfo(f"Executing Task-Space Goal: X={msg.x}, Y={msg.y}, Z={msg.z}")
        
        # 1. Convert XYZ to Joint Angles using your Task 2 math
        target_joints = self.compute_inverse_kinematics(msg.x, msg.y, msg.z)
        
        # 2. Send the joint angles to the controller
        self.send_trajectory([target_joints], duration=3.0)

    def waypoints_callback(self, msg):
        """Accepts an array of Poses, computes IK for each, and executes a continuous path."""
        rospy.loginfo(f"Received {len(msg.poses)} waypoints. Computing path...")
        
        trajectory_points = []
        for pose in msg.poses:
            # Run IK for each waypoint in the list
            joints = self.compute_inverse_kinematics(pose.position.x, pose.position.y, pose.position.z)
            trajectory_points.append(joints)

        # Send the entire list of waypoints to the controller
        # We allocate 2.0 seconds between each waypoint
        self.send_trajectory(trajectory_points, duration=2.0)

    def send_trajectory(self, list_of_joint_arrays, duration=2.0):
        """
        Helper function to build and send the FollowJointTrajectoryGoal.
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        # Build the trajectory points
        time_from_start = 0.0
        for joints in list_of_joint_arrays:
            point = JointTrajectoryPoint()
            point.positions = joints
            
            # Increment time for each waypoint to ensure smooth motion
            time_from_start += duration
            point.time_from_start = rospy.Duration(time_from_start)
            
            goal.trajectory.points.append(point)

        # Send the trajectory to ros_control
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Trajectory execution complete.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        commander = OpenManipulatorDirectCommander()
        commander.run()
    except rospy.ROSInterruptException:
        pass
