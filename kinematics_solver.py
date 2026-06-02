import numpy as np

class OpenManipulatorKinematics:
    def __init__(self):
        # OpenManipulator-X Link Lengths (in meters) [cite: 27]
        self.L1 = 0.0775  # Base to Joint 2 height (Z)
        self.L2 = 0.1300  # Joint 2 to Joint 3 length
        self.L3 = 0.1240  # Joint 3 to Joint 4 length
        self.L4 = 0.1260  # Joint 4 to Gripper tool center point (TCP)

    def forward_kinematics(self, joints):
        """
        Calculates the end-effector (TCP) position (x, y, z) based on 4 joint angles[cite: 30, 53].
        Angles expected in radians[cite: 28].
        """
        q1, q2, q3, q4 = joints

        # Projecting the arm length onto the ground plane
        r = self.L2 * np.cos(q2) + self.L3 * np.cos(q2 + q3) + self.L4 * np.cos(q2 + q3 + q4)
        
        # Absolute coordinates relative to base frame [cite: 29, 30]
        x = r * np.cos(q1)
        y = r * np.sin(q1)
        z = self.L1 + self.L2 * np.sin(q2) + self.L3 * np.sin(q2 + q3) + self.L4 * np.sin(q2 + q3 + q4)
        
        return np.array([x, y, z])

    def inverse_kinematics(self, target_pos):
        """
        Analytically solves IK for a desired (x, y, z) position using trigonometry[cite: 50, 51].
        Assumes the gripper remains parallel to the ground plane (q2 + q3 + q4 = 0) 
        to keep the solution stable and deterministic.
        """
        x, y, z = target_pos

        # 1. Solve Joint 1 (Waist angle looking down from top plane)
        q1 = np.arctan2(y, x)

        # 2. Project target into the 2D plane of the arm (r, z)
        r = np.sqrt(x**2 + y**2)
        
        # Account for link 4 pointing forward horizontally (assuming q2+q3+q4=0)
        r_eff = r - self.L4
        z_eff = z - self.L1

        # 3. Solve Joint 3 using Law of Cosines on the triangle formed by L2 and L3
        D = (r_eff**2 + z_eff**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        
        # Clip D to handle occasional floating-point rounding issues outside [-1, 1]
        D = np.clip(D, -1.0, 1.0)
        
        # Elbow-up configuration (-), use (+) for elbow-down
        q3 = -np.arccos(D)

        # 4. Solve Joint 2
        theta1 = np.arctan2(z_eff, r_eff)
        theta2 = np.arctan2(self.L3 * np.sin(q3), self.L2 + self.L3 * np.cos(q3))
        q2 = theta1 - theta2

        # 5. Solve Joint 4 to maintain horizontal orientation constraint
        q4 = 0.0 - q2 - q3

        return np.array([q1, q2, q3, q4])

# =====================================================================
# Execution and Verification (Task 2 Requirements) [cite: 2]
# =====================================================================
if __name__ == "__main__":
    solver = OpenManipulatorKinematics()

    # 1. Define three reachable target end-effector poses (X, Y, Z in meters) [cite: 27, 50]
    target_poses = [
        np.array([0.25, 0.0, 0.15]),   # Target 1
        np.array([0.18, 0.12, 0.20]),  # Target 2
        np.array([0.20, -0.10, 0.08])  # Target 3
    ]

    print("==================================================")
    print("  OPENMANIPULATOR-X GEOMETRIC KINEMATICS SOLVER   ")
    print("==================================================\n")

    for i, target in enumerate(target_poses, 1):
        print(f"--- POSE {i} ---") [cite: 50]
        print(f"Target Position (X, Y, Z): {target} meters") [cite: 27]
        
        # Solve Inverse Kinematics [cite: 50]
        joint_angles = solver.inverse_kinematics(target)
        print(f"Solved Joint Angles (q1, q2, q3, q4):\n  {np.round(joint_angles, 4)} rad") [cite: 28]
        print(f"  {np.round(np.degrees(joint_angles), 2)} deg")
        
        # Verify using Forward Kinematics [cite: 53]
        verified_pos = solver.forward_kinematics(joint_angles)
        print(f"FK Verification Position:   {np.round(verified_pos, 4)}") [cite: 53]
        
        # Compute tracking error
        error = np.linalg.norm(target - verified_pos)
        print(f"Absolute Position Error:    {error:.6f} meters") [cite: 27]
        print("\n" + "-"*50 + "\n")
