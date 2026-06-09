#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class TaskSpacePolynomialTrajectoryDirect:
    def __init__(self):
        rospy.init_node('task9_polynomial_generator', anonymous=True)

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.current_joints = [0.0, 0.0, 0.0, 0.0]

        # 1. Listen to joint states so we always know the current position
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # 2. Set up the direct controller action client instead of MoveIt
        self.client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for /arm_controller/follow_joint_trajectory action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to trajectory action server!")

        # 3. Listen for a target point to trigger the interpolation execution loop
        rospy.Subscriber('/execute_polynomial_task', Point, self.trajectory_callback)
        rospy.loginfo("Task 9 Polynomial Trajectory Node Operational.")

    def joint_state_callback(self, msg):
        """Updates our local storage of where the robot joints currently are."""
        # Ensure we map the joints accurately based on incoming names
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == 4:
            self.current_joints = positions

    def compute_forward_kinematics(self, joints):
        """
        TODO: Drop your Task 2 Forward Kinematics equations here.
        Takes a list of 4 joint angles and returns the current [X, Y, Z] position.
        """
        # --- YOUR MATH GOES HERE ---
        # Dummy placeholder (Assumes arm is at home/origin for safety):
        x = 0.1
        y = 0.0
        z = 0.1
        return np.array([x, y, z])

    def compute_inverse_kinematics(self, x, y, z):
        """
        TODO: Drop your Task 2 Inverse Kinematics math here.
        Takes an X, Y, Z position and returns a list of 4 joint angles [j1, j2, j3, j4].
        """
        # --- YOUR MATH GOES HERE ---
        # Dummy placeholder:
        j1, j2, j3, j4 = 0.0, 0.0, 0.0, 0.0
        return [j1, j2, j3, j4]

    def compute_cubic_coefficients(self, p_start, p_goal, T):
        """
        Computes cubic polynomial coefficients given start/end positions and total time T.
        Boundary conditions: v(0) = 0, v(T) = 0
        """
        a0 = p_start
        a1 = np.zeros_like(p_start)
        a2 = 3 * (p_goal - p_start) / (T ** 2)
        a3 = -2 * (p_goal - p_start) / (T ** 3)
        return a0, a1, a2, a3

    def trajectory_callback(self, msg):
        rospy.loginfo("Received new target task-space goal. Calculating trajectory coefficients...")
        
        # 1. Use FK to find out where the end-effector currently is right now
        p_start = self.compute_forward_kinematics(self.current_joints)
        p_goal = np.array([msg.x, msg.y, msg.z])

        # 2. Define Time scale parameters
        T = 4.0          # Total execution duration constraint: 4 seconds
        dt = 0.1         # Sample intermediate resolution waypoint every 100ms
        time_steps = np.arange(0, T + dt, dt)

        # Calculate cubic matrices
        a0, a1, a2, a3 = self.compute_cubic_coefficients(p_start, p_goal, T)

        # 3. Formulate the Action Goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        # 4. Step through time, solve cubic positions, run IK, and build our trajectory points
        rospy.loginfo(f"Generating Smooth Cubic Path via {len(time_steps)} calculated steps...")
        
        for t in time_steps:
            # s(t) = a0 + a1*t + a2*t^2 + a3*t^3
            p_t = a0 + a1*t + a2*(t**2) + a3*(t**3)
            
            # Use IK to convert this point along the cubic arc into actual joint angles
            joints_t = self.compute_inverse_kinematics(p_t[0], p_t[1], p_t[2])
            
            # Package it up as a trajectory waypoint
            point = JointTrajectoryPoint()
            point.positions = joints_t
            point.time_from_start = rospy.Duration(t)
            
            goal.trajectory.points.append(point)

        # 5. Execute directly on hardware/simulation links
        rospy.loginfo("Trajectory calculation valid. Initiating Execution tracking window...")
        start_time = rospy.Time.now().to_sec()
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        end_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"Execution wrapped successfully in {end_time - start_time:.2f} seconds.")
        rospy.loginfo("Log tracking data completed. Ready for Rosbag analysis evaluation.")

if __name__ == '__main__':
    try:
        generator = TaskSpacePolynomialTrajectoryDirect()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
