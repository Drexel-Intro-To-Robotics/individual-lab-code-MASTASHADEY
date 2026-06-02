import numpy as np
from scipy.optimize import minimize

class OpenManipulatorKinematics:
    def __init__(self):
        # OpenManipulator-X Link Lengths (in meters)
        # Based on standard Robotis specifications:
        self.L1 = 0.0775  # Base to Joint 2 height (Z)
        self.L2 = 0.1300  # Joint 2 to Joint 3 length
        self.L3 = 0.1240  # Joint 3 to Joint 4 length
        self.L4 = 0.1260  # Joint 4 to Gripper center tool center point (TCP)

        # Joint physical limits (in radians)
        self.bounds = [
            (-np.pi, np.pi),        # Joint 1 (Waist)
            (-0.57 * np.pi, 0.5 * np.pi), # Joint 2
            (-0.5 * np.pi, 0.44 * np.pi),  # Joint 3
            (-0.57 * np.pi, 0.65 * np.pi)  # Joint 4
        ]

    def forward_kinematics(self, joints):
        """
        Calculates the end-effector (TCP) position (x, y, z) based on 4 joint angles.
        Angles expected in radians.
        """
        q1, q2, q3, q4 = joints

        # Standard planar reduction geometry for OpenManipulator-X
        # Projecting the arm length onto the ground plane
        r = self.L2 * np.cos(q2) + self.L3 * np.cos(q2 + q3) + self.L4 * np.cos(q2 + q3 + q4)
        
        # Absolute X, Y coordinates relative to base frame
        x = r * np.cos(q1)
        y = r * np.sin(q1)
        
        # Absolute Z coordinate relative to base frame
        z = self.L1 + self.L2 * np.sin(q2) + self.L3 * np.sin(q2 + q3) + self.L4 * np.sin(q2 + q3 + q4)
        
        return np.array([x, y, z])

    def loss_function(self, joints, target_pos):
        """Calculates squared Euclidean distance between current and target position."""
        current_pos = self.forward_kinematics(joints)
        return np.sum((current_pos - target_pos) ** 2)

    def inverse_kinematics(self, target_pos, initial_guess=None):
        """
        Numerically solves IK for a desired (x, y, z) position using optimization.
        """
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0] # Default home position guess

        # Run optimization minimizing position error within joint boundaries
        result = minimize(
            self.loss_function, 
            initial_guess, 
            args=(target_pos,), 
            bounds=self.bounds,
            method='SLSQP'
        )
        
        if result.success:
            return result.x
        else:
            raise ValueError(f"IK Optimization failed to converge for target {target_pos}")

# =====================================================================
# Execution and Verification (Task 2 Requirements)
# =====================================================================
if __name__ == "__main__":
    solver = OpenManipulatorKinematics()

    # 1. Define three reachable target end-effector poses (X, Y, Z in meters)
    target_poses = [
        np.array([0.25, 0.0, 0.15]),   # Target 1: Straight ahead, slightly raised
        np.array([0.18, 0.12, 0.20]),  # Target 2: Reaching to the left and up
        np.array([0.20, -0.10, 0.08])  # Target 3: Reaching to the right and lower
    ]

    print("==================================================")
    print("      OPENMANIPULATOR-X KINEMATICS SOLVER         ")
    print("==================================================\n")

    for i, target in enumerate(target_poses, 1):
        print(f"--- POSE {i} ---")
        print(f"Target Position (X, Y, Z): {target} meters")
        
        # Solve Inverse Kinematics
        try:
            joint_angles = solver.inverse_kinematics(target)
            print(f"Solved Joint Angles (q1, q2, q3, q4):\n  {np.round(joint_angles, 4)} rad")
            print(f"  {np.round(np.degrees(joint_angles), 2)} deg")
            
            # Verify using Forward Kinematics
            verified_pos = solver.forward_kinematics(joint_angles)
            print(f"FK Verification Position:   {np.round(verified_pos, 4)}")
            
            # Compute tracking error
            error = np.linalg.norm(target - verified_pos)
            print(f"Absolute Position Error:    {error:.6f} meters")
            
        except ValueError as e:
            print(f"Error: {e}")
        print("\n" + "-"*50 + "\n")
