#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Point

class OpenManipulatorGoalCommander:
    def __init__(self):
        # Initialize moveit_commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('task8_goal_commander', anonymous=True)

        # Instantiate MoveGroupCommander for the arm
        # Note: "arm" is the default group name for OpenManipulator-X. Change if your setup differs.
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        # Set up subscribers for the three types of goals
        rospy.Subscriber('/goal_joint_space', Float64MultiArray, self.joint_space_callback)
        rospy.Subscriber('/goal_task_space', Point, self.task_space_callback)
        rospy.Subscriber('/goal_waypoints', geometry_msgs.msg.PoseArray, self.waypoints_callback)

        rospy.loginfo("Task 8 Goal Commander Node Operational. Awaiting goals...")

    def joint_space_callback(self, msg):
        """Accepts an array of 4 joint angles in radians and executes."""
        if len(msg.data) != 4:
            rospy.logwarn("Joint space goal requires exactly 4 angles!")
            return
        
        rospy.loginfo(f"Executing Joint-Space Goal: {msg.data}")
        self.move_group.go(msg.data, wait=True)
        self.move_group.stop()

    def task_space_callback(self, msg):
        """Accepts an X, Y, Z coordinate point and plans/executes task-space move."""
        rospy.loginfo(f"Executing Task-Space Goal: X={msg.x}, Y={msg.y}, Z={msg.z}")
        
        # Get current orientation to preserve it, updating only position
        current_pose = self.move_group.get_current_pose().pose
        target_pose = Pose()
        target_pose.position.x = msg.x
        target_pose.position.y = msg.y
        target_pose.position.z = msg.z
        target_pose.orientation = current_pose.orientation # Maintain stable orientation

        self.move_group.set_pose_target(target_pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def waypoints_callback(self, msg):
        """Accepts an array of Poses and computes a continuous Cartesian path."""
        rospy.loginfo(f"Received a list of {len(msg.poses)} waypoints. Computing Cartesian path...")
        
        waypoints = []
        for pose in msg.poses:
            waypoints.append(pose)

        # Compute Cartesian trajectory line
        # fraction represents the percentage of the path successfully planned (0.0 to 1.0)
        (plan, fraction) = self.move_group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step (1cm resolution)
                             0.0)         # jump_threshold

        if fraction > 0.90: # Ensure at least 90% of path is viable
            rospy.loginfo(f"Path planning successful ({fraction*100:.1f}%). Executing...")
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logerr(f"Cartesian path rejected. Only {fraction*100:.1f}% of path is clear of obstacles.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        commander = OpenManipulatorGoalCommander()
        commander.run()
    except rospy.ROSInterruptException:
        pass