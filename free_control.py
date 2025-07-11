'''
Free Control UR5e Simulation - Manual Joint Control
This script allows you to interactively control the UR5e robot joints and explore the scene.
'''

import numpy as np
import time
import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def print_controls():
    """Print control instructions"""
    print("\n" + "="*60)
    print("UR5e FREE CONTROL SIMULATION")
    print("="*60)
    print("CONTROLS:")
    print("  Q/A - Shoulder Pan Joint (Joint 0)")
    print("  W/S - Shoulder Lift Joint (Joint 1)")
    print("  E/D - Elbow Joint (Joint 2)")
    print("  R/F - Wrist 1 Joint (Joint 3)")
    print("  T/G - Wrist 2 Joint (Joint 4)")
    print("  Y/H - Wrist 3 Joint (Joint 5)")
    print()
    print("  1 - Home Position")
    print("  2 - Ready Position")
    print("  3 - Zero Position")
    print("  0 - Reset to Current Position")
    print()
    print("  P - Print Current Joint Positions")
    print("  I - Print End Effector Pose")
    print("  C - Print Controls Again")
    print("  ESC/X - Exit")
    print("="*60)
    print("Move the robot by holding down the keys!")
    print("Joint angles will be shown in real-time.\n")

def free_control_simulation():
    """
    Free control simulation with keyboard input
    """
    
    try:
        from src.ur5e_simulator import UR5eSimulator
        
        print("Initializing UR5e Free Control Simulation...")
        ur5e = UR5eSimulator(
            name='UR5e_FreeControl',
            xml_path='ur5e_main.xml',
            USE_VIEWER=True,
            VERBOSE=False  # Reduce verbosity for cleaner output
        )
        
        print("✓ UR5e simulator initialized!")
        print(f"  Number of joints: {ur5e.n_joints}")
        print(f"  Joint names: {ur5e.joint_names}")
        
        # Print controls
        print_controls()
        
        # Initialize joint positions
        current_joints = np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])  # Ready position
        ur5e.set_joint_positions(current_joints)
        
        # Joint limits (approximate for UR5e)
        joint_limits = np.array([
            [-6.28, 6.28],   # Shoulder pan
            [-6.28, 6.28],   # Shoulder lift
            [-3.14, 3.14],   # Elbow
            [-6.28, 6.28],   # Wrist 1
            [-6.28, 6.28],   # Wrist 2
            [-6.28, 6.28]    # Wrist 3
        ])
        
        # Control parameters
        joint_speed = 0.02  # radians per step
        update_counter = 0
        
        # Predefined positions
        home_position = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
        ready_position = np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])
        zero_position = np.zeros(6)
        
        print("Free control active! Use keyboard controls...")
        print("Current joint positions (deg):", np.rad2deg(current_joints))
        
        # Main control loop
        try:
            import msvcrt  # Windows-specific keyboard input
            
            while True:
                # Check for keyboard input (non-blocking)
                key_pressed = False
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8').lower()
                    key_pressed = True
                    
                    # Joint controls
                    if key == 'q':
                        current_joints[0] = np.clip(current_joints[0] + joint_speed, 
                                                   joint_limits[0,0], joint_limits[0,1])
                    elif key == 'a':
                        current_joints[0] = np.clip(current_joints[0] - joint_speed, 
                                                   joint_limits[0,0], joint_limits[0,1])
                    elif key == 'w':
                        current_joints[1] = np.clip(current_joints[1] + joint_speed, 
                                                   joint_limits[1,0], joint_limits[1,1])
                    elif key == 's':
                        current_joints[1] = np.clip(current_joints[1] - joint_speed, 
                                                   joint_limits[1,0], joint_limits[1,1])
                    elif key == 'e':
                        current_joints[2] = np.clip(current_joints[2] + joint_speed, 
                                                   joint_limits[2,0], joint_limits[2,1])
                    elif key == 'd':
                        current_joints[2] = np.clip(current_joints[2] - joint_speed, 
                                                   joint_limits[2,0], joint_limits[2,1])
                    elif key == 'r':
                        current_joints[3] = np.clip(current_joints[3] + joint_speed, 
                                                   joint_limits[3,0], joint_limits[3,1])
                    elif key == 'f':
                        current_joints[3] = np.clip(current_joints[3] - joint_speed, 
                                                   joint_limits[3,0], joint_limits[3,1])
                    elif key == 't':
                        current_joints[4] = np.clip(current_joints[4] + joint_speed, 
                                                   joint_limits[4,0], joint_limits[4,1])
                    elif key == 'g':
                        current_joints[4] = np.clip(current_joints[4] - joint_speed, 
                                                   joint_limits[4,0], joint_limits[4,1])
                    elif key == 'y':
                        current_joints[5] = np.clip(current_joints[5] + joint_speed, 
                                                   joint_limits[5,0], joint_limits[5,1])
                    elif key == 'h':
                        current_joints[5] = np.clip(current_joints[5] - joint_speed, 
                                                   joint_limits[5,0], joint_limits[5,1])
                    
                    # Predefined positions
                    elif key == '1':
                        current_joints = home_position.copy()
                        print("Moved to HOME position")
                    elif key == '2':
                        current_joints = ready_position.copy()
                        print("Moved to READY position")
                    elif key == '3':
                        current_joints = zero_position.copy()
                        print("Moved to ZERO position")
                    elif key == '0':
                        print("Position reset")
                    
                    # Information commands
                    elif key == 'p':
                        print(f"Joint positions (deg): {np.rad2deg(current_joints)}")
                        print(f"Joint positions (rad): {current_joints}")
                    elif key == 'i':
                        ee_pos, ee_rot = ur5e.get_end_effector_pose()
                        if ee_pos is not None:
                            print(f"End effector position: {ee_pos}")
                            print(f"End effector orientation (3x3): \n{ee_rot}")
                        else:
                            print("Could not get end effector pose")
                    elif key == 'c':
                        print_controls()
                    
                    # Exit commands
                    elif key == 'x' or ord(key) == 27:  # 'x' or ESC
                        print("Exiting free control simulation...")
                        break
                
                # Update robot position
                ur5e.set_joint_positions(current_joints)
                
                # Render simulation
                ur5e.render()
                
                # Print joint positions periodically
                if key_pressed or (update_counter % 100 == 0):
                    joint_degrees = np.rad2deg(current_joints)
                    print(f"Joints (deg): [{joint_degrees[0]:6.1f}, {joint_degrees[1]:6.1f}, {joint_degrees[2]:6.1f}, {joint_degrees[3]:6.1f}, {joint_degrees[4]:6.1f}, {joint_degrees[5]:6.1f}]", end='\r')
                
                update_counter += 1
                time.sleep(0.01)  # Small delay for smooth operation
                
        except ImportError:
            print("msvcrt not available (Windows only). Using alternative method...")
            
            # Alternative method without keyboard input
            print("\nAlternative Mode: Automatic demonstration")
            print("The robot will move through various poses automatically.")
            print("Close the viewer window to exit.")
            
            positions = [ready_position, home_position, zero_position]
            position_names = ["READY", "HOME", "ZERO"]
            
            pos_idx = 0
            while True:
                target_pos = positions[pos_idx]
                print(f"\nMoving to {position_names[pos_idx]} position...")
                
                # Smooth transition
                for i in range(100):
                    alpha = i / 99.0
                    interpolated = (1 - alpha) * current_joints + alpha * target_pos
                    ur5e.set_joint_positions(interpolated)
                    ur5e.render()
                    time.sleep(0.02)
                
                current_joints = target_pos.copy()
                
                # Hold position
                for i in range(200):
                    ur5e.render()
                    time.sleep(0.01)
                
                pos_idx = (pos_idx + 1) % len(positions)
        
        # Close simulator
        ur5e.close_viewer()
        print("\n\n✓ Free control simulation completed!")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure MuJoCo is installed: pip install mujoco")
        return False
    except Exception as e:
        print(f"Error occurred: {e}")
        return False
    
    return True

if __name__ == '__main__':
    print("=" * 60)
    print("UR5e FREE CONTROL SIMULATION")
    print("=" * 60)
    
    if free_control_simulation():
        print("\n" + "=" * 60)
        print("✓ Free control simulation completed successfully!")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("✗ Simulation failed. Please check the error messages above.")
        print("=" * 60)
