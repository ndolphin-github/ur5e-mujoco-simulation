'''
Example usage of UR5e simulator
'''

import numpy as np
import time
from src.ur5e_simulator import UR5eSimulator

def main():
    """
        Main function demonstrating UR5e simulation
    """
    
    # Initialize UR5e simulator
    print("Initializing UR5e simulator...")
    ur5e = UR5eSimulator(
        name='UR5e_Robot',
        xml_path='ur5e_main.xml',
        USE_VIEWER=True,
        VERBOSE=True
    )
    
    # Define some test joint positions (6 joints for UR5e)
    home_position = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
    ready_position = np.array([0, -1.5708, 0, -1.5708, 0, 0])
    
    print("\n=== Demo 1: Moving to Home Position ===")
    ur5e.set_joint_positions(home_position)
    
    # Get current end effector pose
    ee_pos, ee_rot = ur5e.get_end_effector_pose()
    if ee_pos is not None:
        print(f"End effector position: {ee_pos}")
        print(f"End effector rotation shape: {ee_rot.shape}")
    
    # Render for a while
    for i in range(100):
        ur5e.render()
        time.sleep(0.01)
    
    print("\n=== Demo 2: Moving to Ready Position ===")
    ur5e.set_joint_positions(ready_position)
    
    # Render for a while
    for i in range(100):
        ur5e.render()
        time.sleep(0.01)
    
    print("\n=== Demo 3: Simple Joint Space Trajectory ===")
    # Interpolate between positions
    n_steps = 100
    for i in range(n_steps):
        alpha = i / (n_steps - 1)
        interpolated_pos = (1 - alpha) * ready_position + alpha * home_position
        ur5e.set_joint_positions(interpolated_pos)
        ur5e.render()
        time.sleep(0.02)
    
    print("\n=== Demo 4: Inverse Kinematics Test ===")
    # Define a target position for the end effector
    target_position = np.array([0.3, 0.3, 0.5])  # Example target
    
    print(f"Target position: {target_position}")
    
    # Try to add visual marker at target position
    marker_added = ur5e.add_visual_marker(target_position, size=0.03, color=[0, 1, 0, 1])
    if not marker_added:
        print("Note: Visual marker not supported in this MuJoCo version")
    
    # Solve IK
    ik_solution = ur5e.inverse_kinematics(target_position)
    if ik_solution is not None:
        print(f"IK solution: {ik_solution}")
        
        # Get actual end effector position after IK
        final_pos, _ = ur5e.get_end_effector_pose()
        if final_pos is not None:
            error = np.linalg.norm(final_pos - target_position)
            print(f"Final end effector position: {final_pos}")
            print(f"Position error: {error:.6f}")
    
    # Render final result
    for i in range(200):
        ur5e.render()
        time.sleep(0.01)
    
    print("\n=== Demo 5: Control with Actuators ===")
    # Reset to home position
    ur5e.set_joint_positions(home_position)
    
    # Simple sinusoidal movement of joint 0
    for i in range(200):
        t = i * 0.02
        joint_cmd = home_position.copy()
        joint_cmd[0] = home_position[0] + 0.5 * np.sin(t)  # Move shoulder joint
        
        # Use step function with control input
        ur5e.step(ctrl=joint_cmd)
        ur5e.render()
        time.sleep(0.02)
    
    print("\nDemo completed! Press any key to exit...")
    input()
    
    # Close simulator
    ur5e.close_viewer()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error occurred: {e}")
        print("Make sure MuJoCo is installed and the XML files are properly set up.")
