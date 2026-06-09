#!/usr/bin/env python3

import rospy
import actionlib
import geometry_msgs.msg

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectoryPoint

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Import the native ROS Inverse Kinematics Service
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


class OpenManipulatorTopicCommander:
    def __init__(self):
        rospy.init_node('task8_direct_commander', anonymous=True)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # 1. Connect to the arm trajectory ACTION server
        # Important: use action name WITHOUT /goal
        self.traj_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        rospy.loginfo("Waiting for /arm_controller/follow_joint_trajectory action server...")
        self.traj_client.wait_for_server()
        rospy.loginfo("Connected to arm trajectory action server!")

        # 2. Connect to the native ROS Inverse Kinematics Service
        rospy.loginfo("Waiting for /compute_ik service...")
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        rospy.loginfo("Connected to IK service!")

        # 3. Subscribers for Task 8 requirements
        rospy.Subscriber('/goal_joint_space', Float64MultiArray, self.joint_space_callback)
        rospy.Subscriber('/goal_task_space', Point, self.task_space_callback)
        rospy.Subscriber('/goal_waypoints', geometry_msgs.msg.PoseArray, self.waypoints_callback)

        rospy.loginfo("Task 8 Action Commander Operational. Awaiting goals...")

    def compute_inverse_kinematics(self, x, y, z):
        """
        Uses the native ROS MoveIt /compute_ik service to calculate IK.
        Input: desired end-effector position x, y, z.
        Output: list of 4 joint angles, or None if IK fails.
        """

        req = GetPositionIKRequest()

        req.ik_request.group_name = "arm"

        # Starting seed for IK
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0]

        # Use the planning/base frame.
        # If "world" fails, try "base_link".
        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.pose_stamped.header.stamp = rospy.Time.now()

        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z

        # Neutral orientation
        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 1.0

        req.ik_request.timeout = rospy.Duration(1.0)

        try:
            resp = self.ik_service(req)

            if resp.error_code.val == 1:
                joints = []

                for name in self.joint_names:
                    if name not in resp.solution.joint_state.name:
                        rospy.logerr(f"Joint {name} not found in IK solution.")
                        rospy.logerr(f"IK returned joints: {resp.solution.joint_state.name}")
                        return None

                    idx = resp.solution.joint_state.name.index(name)
                    joints.append(resp.solution.joint_state.position[idx])

                rospy.loginfo(f"IK solution found: {joints}")
                return joints

            else:
                rospy.logwarn(f"IK Service failed. Error code: {resp.error_code.val}")
                rospy.logwarn("Target may be out of reach, wrong frame, or bad orientation.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr(f"IK Service call failed: {e}")
            return None

    def joint_space_callback(self, msg):
        """
        Executes a direct list of 4 joint angles.
        Example:
        rostopic pub -1 /goal_joint_space std_msgs/Float64MultiArray "data: [0.0, -0.5, 0.5, 0.0]"
        """

        if len(msg.data) != 4:
            rospy.logwarn("Joint space goal requires exactly 4 angles!")
            return

        rospy.loginfo(f"Executing Joint-Space Goal: {list(msg.data)}")
        self.send_trajectory([list(msg.data)], duration=3.0)

    def task_space_callback(self, msg):
        """
        Takes an X, Y, Z coordinate, runs IK, and executes.
        Example:
        rostopic pub -1 /goal_task_space geometry_msgs/Point "{x: 0.10, y: 0.10, z: 0.15}"
        """

        rospy.loginfo(f"Executing Task-Space Goal: X={msg.x}, Y={msg.y}, Z={msg.z}")

        target_joints = self.compute_inverse_kinematics(msg.x, msg.y, msg.z)

        if target_joints is not None:
            self.send_trajectory([target_joints], duration=3.0)
        else:
            rospy.logwarn("No trajectory sent because IK failed.")

    def waypoints_callback(self, msg):
        """
        Accepts an array of Poses, computes IK for each,
        and executes a continuous waypoint trajectory.
        """

        rospy.loginfo(f"Received {len(msg.poses)} waypoints. Computing IK path...")

        trajectory_points = []

        for i, pose in enumerate(msg.poses):
            joints = self.compute_inverse_kinematics(
                pose.position.x,
                pose.position.y,
                pose.position.z
            )

            if joints is not None:
                trajectory_points.append(joints)
            else:
                rospy.logerr(f"Failed to calculate IK for waypoint {i}. Aborting.")
                return

        self.send_trajectory(trajectory_points, duration=2.0)

    def send_trajectory(self, list_of_joint_arrays, duration=2.0):
        """
        Sends a FollowJointTrajectoryGoal to the arm controller action server.
        This replaces publishing directly to /arm_controller/command.
        """

        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = self.joint_names

        # Start slightly in the future to avoid timing issues
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

        time_from_start = 0.0

        for joints in list_of_joint_arrays:
            point = JointTrajectoryPoint()
            point.positions = list(joints)

            # Optional: leave velocity/acceleration empty
            point.velocities = []
            point.accelerations = []

            time_from_start += duration
            point.time_from_start = rospy.Duration(time_from_start)

            goal.trajectory.points.append(point)

        rospy.loginfo("Sending trajectory goal to /arm_controller/follow_joint_trajectory...")
        self.traj_client.send_goal(goal)

        finished = self.traj_client.wait_for_result(rospy.Duration(time_from_start + 5.0))

        if finished:
            result = self.traj_client.get_result()
            rospy.loginfo(f"Trajectory execution finished. Result: {result}")
        else:
            rospy.logwarn("Trajectory action did not finish before timeout.")
            self.traj_client.cancel_goal()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        commander = OpenManipulatorTopicCommander()
        commander.run()
    except rospy.ROSInterruptException:
        pass
