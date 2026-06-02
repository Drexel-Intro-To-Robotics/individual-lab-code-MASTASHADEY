#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import numpy as np
from geometry_msgs.msg import Point, Pose

class TaskSpacePolynomialTrajectory:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('task9_polynomial_generator', anonymous=True)

        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # Listen for a target point to trigger the interpolation execution loop
        rospy.Subscriber('/execute_polynomial_task', Point, self.trajectory_callback)
        rospy.loginfo("Task 9 Polynomial Trajectory Node Operational.")

    def compute_cubic_coefficients(self, p_start, p_goal, T):
        """
        Computes cubic polynomial coefficients given start/end positions and total time T.
        Boundary conditions: v(0) = 0, v(T) = 0
        """
        # a0 = p_start
        # a1 = 0
        # a2 = 3*(p_goal - p_start) / T^2
        # a3 = -2*(p_goal - p_start) / T^3
        a0 = p_start
        a1 = np.zeros_like(p_start)
        a2 = 3 * (p_goal - p_start) / (T ** 2)
        a3 = -2 * (p_goal - p_start) / (T ** 3)
        return a0, a1, a2, a3

    def trajectory_callback(self, msg):
        rospy.loginfo("Received new target task-space goal. Calculating trajectory coefficients...")
        
        # 1. Get starting configuration parameters
        start_pose = self.move_group.get_current_pose().pose
        p_start = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        p_goal = np.array([msg.x, msg.y, msg.z])

        # 2. Define Time scale parameters
        T = 4.0          # Total execution duration constraint: 4 seconds
        dt = 0.1         # Sample intermediate resolution waypoint every 100ms
        time_steps = np.arange(0, T + dt, dt)

        # Calculate cubic matrices
        a0, a1, a2, a3 = self.compute_cubic_coefficients(p_start, p_goal, T)

        # 3. Generate intermediate sampled Cartesian waypoint vector
        waypoints = []
        for t in time_steps:
            # s(t) = a0 + a1*t + a2*t^2 + a3*t^3 [cite: 96]
            p_t = a0 + a1*t + a2*(t**2) + a3*(t**3)
            
            w_pose = Pose()
            w_pose.position.x = p_t[0]
            w_pose.position.y = p_t[1]
            w_pose.position.z = p_t[2]
            w_pose.orientation = start_pose.orientation # Keep end-effector level
            waypoints.append(w_pose)

        # 4. Compile Waypoints into a Cartesion Plan profile
        rospy.loginfo(f"Generating Smooth Cubic Path via {len(waypoints)} calculated steps...")
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        if fraction > 0.95:
            rospy.loginfo("Trajectory calculation valid. Initiating Execution tracking window...")
            
            # Record starting timestamp context
            start_time = rospy.Time.now().to_sec()
            
            # Execute plan profile on hardware/simulation links [cite: 99]
            self.move_group.execute(plan, wait=True)
            
            end_time = rospy.Time.now().to_sec()
            rospy.loginfo(f"Execution wrapped successfully in {end_time - start_time:.2f} seconds.")
            rospy.loginfo("Log tracking data completed. Ready for Rosbag analysis evaluation.") [cite: 124]
        else:
            rospy.logerr("Unable to safely interpolate polynomial path without risking kinematic bounds limits.")

if __name__ == '__main__':
    try:
        generator = TaskSpacePolynomialTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    