'''
Free Control UR5e + RH-P12-RN Gripper Simulation - Manual Joint & Gripper Control
This script allows you to interactively control the UR5e robot joints and RH-P12-RN gripper.
'''

import numpy as np
import time
import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def print_controls():
    """Print control instructions"""
    print("\n" + "="*70)
    print("UR5e + RH-P12-RN GRIPPER FREE CONTROL SIMULATION")
    print("="*70)
    print("ROBOT ARM CONTROLS:")
    print("  Q/A - Shoulder Pan Joint (Joint 0)")
    print("  W/S - Shoulder Lift Joint (Joint 1)")
    print("  E/D - Elbow Joint (Joint 2)")
    print("  R/F - Wrist 1 Joint (Joint 3)")
    print("  T/G - Wrist 2 Joint (Joint 4)")
    print("  Y/H - Wrist 3 Joint (Joint 5)")
    print()
    print("GRIPPER CONTROLS:")
    print("  U/J - Right Finger (Joint 6)")
    print("  I/K - Left Finger (Joint 7)")
    print("  O/L - Right Finger Tip (Joint 8)")
    print("  P/; - Left Finger Tip (Joint 9)")
    print("  SPACE - Open Gripper")
    print("  ENTER - Close Gripper")
    print()
    print("PREDEFINED POSITIONS:")
    print("  1 - Home Position")
    print("  2 - Ready Position")
    print("  3 - Zero Position")
    print("  4 - Pick Pose")
    print("  5 - Place Pose")
    print("  0 - Reset to Current Position")
    print()
    print("INFORMATION:")
    print("  V - Print Current Joint Positions")
    print("  B - Print End Effector Pose")
    print("  N - Print Gripper State")
    print("  C - Print Controls Again")
    print("  ESC/X - Exit")
    print("="*70)
    print("Move the robot and gripper by holding down the keys!")
    print("Joint angles will be shown in real-time.\n")

def free_control_with_gripper():
    """
    Free control simulation with keyboard input for UR5e + RH-P12-RN gripper
    """
    
    try:
        from src.ur5e_simulator import UR5eSimulator
        
        print("Initializing UR5e + RH-P12-RN Gripper Simulation...")
        ur5e = UR5eSimulator(
            name='UR5e_Gripper_FreeControl',
            xml_path='ur5e_with_gripper.xml',
            USE_VIEWER=True,
            VERBOSE=False  # Reduce verbosity for cleaner output
        )
        
        print("✓ UR5e + Gripper simulator initialized!")
        print(f"  Total number of joints: {ur5e.n_joints}")
        print(f"  Joint names: {ur5e.joint_names}")
        
        # Print controls
        print_controls()
        
        # Initialize joint positions (6 UR5e joints + 4 gripper joints)
        current_joints = np.array([
            0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0,  # UR5e ready position
            0.0, 0.0, 0.0, 0.0                      # Gripper open position
        ])
        # Set initial position directly (for initialization only)
        ur5e.set_joint_positions(current_joints)
        
        # Joint limits
        joint_limits = np.array([
            # UR5e limits
            [-6.28, 6.28],   # Shoulder pan
            [-6.28, 6.28],   # Shoulder lift
            [-3.14, 3.14],   # Elbow
            [-6.28, 6.28],   # Wrist 1
            [-6.28, 6.28],   # Wrist 2
            [-6.28, 6.28],   # Wrist 3
            # RH-P12-RN Gripper limits
            [0.0, 1.1],      # Right finger
            [0.0, 1.1],      # Left finger
            [0.0, 1.0],      # Right finger tip
            [0.0, 1.0]       # Left finger tip
        ])
        
        # Control parameters
        arm_speed = 0.01      # radians per step for arm joints (reduced for smoother physics)
        gripper_speed = 0.02  # radians per step for gripper joints (reduced for smoother physics)
        update_counter = 0
        
        # Predefined positions
        home_position = np.array([
            -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0,  # UR5e home
            0.0, 0.0, 0.0, 0.0                               # Gripper open
        ])
        ready_position = np.array([
            0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0,           # UR5e ready
            0.0, 0.0, 0.0, 0.0                               # Gripper open
        ])
        zero_position = np.zeros(10)  # All joints to zero
        
        # Pick and place positions
        pick_position = np.array([
            0.0, -1.0, 0.5, -2.0, 0.0, 0.0,                 # UR5e pick pose
            0.0, 0.0, 0.0, 0.0                               # Gripper open
        ])
        place_position = np.array([
            1.57, -1.2, 0.8, -1.8, 0.0, 0.0,                # UR5e place pose
            1.0, 1.0, 0.8, 0.8                               # Gripper closed
        ])
        
        # Gripper presets
        gripper_open = np.array([0.0, 0.0, 0.0, 0.0])
        gripper_closed = np.array([1.0, 1.0, 0.8, 0.8])
        
        print("Free control active! Use keyboard controls...")
        print("Using PHYSICS-BASED CONTROL - collisions will be respected!")
        print("Current joint positions (deg):", np.rad2deg(current_joints))
        
        # Main control loop
        try:
            import msvcrt  # Windows-specific keyboard input
            
            while True:
                # Check for keyboard input (non-blocking)
                key_pressed = False
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    
                    # Handle special keys
                    if key == b'\x00' or key == b'\xe0':  # Special key prefix
                        key = msvcrt.getch()  # Get the actual key
                        continue
                    
                    try:
                        key = key.decode('utf-8').lower()
                    except:
                        continue
                        
                    key_pressed = True
                    
                    # UR5e Joint controls
                    if key == 'q':
                        current_joints[0] = np.clip(current_joints[0] + arm_speed, 
                                                   joint_limits[0,0], joint_limits[0,1])
                    elif key == 'a':
                        current_joints[0] = np.clip(current_joints[0] - arm_speed, 
                                                   joint_limits[0,0], joint_limits[0,1])
                    elif key == 'w':
                        current_joints[1] = np.clip(current_joints[1] + arm_speed, 
                                                   joint_limits[1,0], joint_limits[1,1])
                    elif key == 's':
                        current_joints[1] = np.clip(current_joints[1] - arm_speed, 
                                                   joint_limits[1,0], joint_limits[1,1])
                    elif key == 'e':
                        current_joints[2] = np.clip(current_joints[2] + arm_speed, 
                                                   joint_limits[2,0], joint_limits[2,1])
                    elif key == 'd':
                        current_joints[2] = np.clip(current_joints[2] - arm_speed, 
                                                   joint_limits[2,0], joint_limits[2,1])
                    elif key == 'r':
                        current_joints[3] = np.clip(current_joints[3] + arm_speed, 
                                                   joint_limits[3,0], joint_limits[3,1])
                    elif key == 'f':
                        current_joints[3] = np.clip(current_joints[3] - arm_speed, 
                                                   joint_limits[3,0], joint_limits[3,1])
                    elif key == 't':
                        current_joints[4] = np.clip(current_joints[4] + arm_speed, 
                                                   joint_limits[4,0], joint_limits[4,1])
                    elif key == 'g':
                        current_joints[4] = np.clip(current_joints[4] - arm_speed, 
                                                   joint_limits[4,0], joint_limits[4,1])
                    elif key == 'y':
                        current_joints[5] = np.clip(current_joints[5] + arm_speed, 
                                                   joint_limits[5,0], joint_limits[5,1])
                    elif key == 'h':
                        current_joints[5] = np.clip(current_joints[5] - arm_speed, 
                                                   joint_limits[5,0], joint_limits[5,1])
                    
                    # Gripper controls
                    elif key == 'u':  # Right finger open
                        current_joints[6] = np.clip(current_joints[6] + gripper_speed, 
                                                   joint_limits[6,0], joint_limits[6,1])
                    elif key == 'j':  # Right finger close
                        current_joints[6] = np.clip(current_joints[6] - gripper_speed, 
                                                   joint_limits[6,0], joint_limits[6,1])
                    elif key == 'i':  # Left finger open
                        current_joints[7] = np.clip(current_joints[7] + gripper_speed, 
                                                   joint_limits[7,0], joint_limits[7,1])
                    elif key == 'k':  # Left finger close
                        current_joints[7] = np.clip(current_joints[7] - gripper_speed, 
                                                   joint_limits[7,0], joint_limits[7,1])
                    elif key == 'o':  # Right finger tip
                        current_joints[8] = np.clip(current_joints[8] + gripper_speed, 
                                                   joint_limits[8,0], joint_limits[8,1])
                    elif key == 'l':
                        current_joints[8] = np.clip(current_joints[8] - gripper_speed, 
                                                   joint_limits[8,0], joint_limits[8,1])
                    elif key == 'p':  # Left finger tip
                        current_joints[9] = np.clip(current_joints[9] + gripper_speed, 
                                                   joint_limits[9,0], joint_limits[9,1])
                    elif key == ';':
                        current_joints[9] = np.clip(current_joints[9] - gripper_speed, 
                                                   joint_limits[9,0], joint_limits[9,1])
                    
                    # Gripper presets
                    elif key == ' ':  # Space - Open gripper
                        current_joints[6:10] = gripper_open
                        print("Gripper OPENED")
                    elif key == '\r':  # Enter - Close gripper
                        current_joints[6:10] = gripper_closed
                        print("Gripper CLOSED")
                    
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
                    elif key == '4':
                        current_joints = pick_position.copy()
                        print("Moved to PICK position")
                    elif key == '5':
                        current_joints = place_position.copy()
                        print("Moved to PLACE position")
                    elif key == '0':
                        print("Position reset")
                    
                    # Information commands
                    elif key == 'v':  # Changed from 'p' to avoid conflict
                        arm_joints = current_joints[:6]
                        gripper_joints = current_joints[6:]
                        print(f"ARM joints (deg): {np.rad2deg(arm_joints)}")
                        print(f"ARM joints (rad): {arm_joints}")
                        print(f"GRIPPER joints (deg): {np.rad2deg(gripper_joints)}")
                        print(f"GRIPPER joints (rad): {gripper_joints}")
                    elif key == 'b':  # Changed from 'i' to avoid conflict
                        ee_pos, ee_rot = ur5e.get_end_effector_pose()
                        if ee_pos is not None:
                            print(f"End effector position: {ee_pos}")
                            print(f"End effector orientation (3x3): \n{ee_rot}")
                        else:
                            print("Could not get end effector pose")
                    elif key == 'n':  # Gripper state
                        gripper_joints = current_joints[6:]
                        gripper_openness = 1.0 - np.mean(gripper_joints[:2] / 1.1)  # Main fingers
                        print(f"Gripper state: {gripper_openness*100:.1f}% open")
                        print(f"Right finger: {gripper_joints[0]:.3f} rad ({np.rad2deg(gripper_joints[0]):.1f}°)")
                        print(f"Left finger: {gripper_joints[1]:.3f} rad ({np.rad2deg(gripper_joints[1]):.1f}°)")
                    elif key == 'c':
                        print_controls()
                    
                    # Exit commands
                    elif key == 'x' or ord(key) == 27:  # 'x' or ESC
                        print("Exiting free control simulation...")
                        break
                
                # Update robot position using physics-based control (respects collisions)
                ur5e.set_joint_targets(current_joints)
                
                # Render simulation
                ur5e.render()
                
                # Print joint positions periodically
                if key_pressed or (update_counter % 100 == 0):
                    arm_degrees = np.rad2deg(current_joints[:6])
                    gripper_degrees = np.rad2deg(current_joints[6:])
                    print(f"ARM: [{arm_degrees[0]:6.1f}, {arm_degrees[1]:6.1f}, {arm_degrees[2]:6.1f}, {arm_degrees[3]:6.1f}, {arm_degrees[4]:6.1f}, {arm_degrees[5]:6.1f}] | GRIP: [{gripper_degrees[0]:5.1f}, {gripper_degrees[1]:5.1f}, {gripper_degrees[2]:5.1f}, {gripper_degrees[3]:5.1f}]", end='\r')
                
                update_counter += 1
                time.sleep(0.01)  # Small delay for smooth operation
                
        except ImportError:
            print("msvcrt not available (Windows only). Using alternative method...")
            
            # Alternative method without keyboard input
            print("\nAlternative Mode: Automatic demonstration")
            print("The robot will move through various poses automatically.")
            print("Close the viewer window to exit.")
            
            positions = [ready_position, pick_position, place_position, home_position]
            position_names = ["READY", "PICK", "PLACE", "HOME"]
            
            pos_idx = 0
            while True:
                target_pos = positions[pos_idx]
                print(f"\nMoving to {position_names[pos_idx]} position...")
                
                # Smooth transition using physics-based control
                for i in range(150):  # Longer transition for gripper
                    alpha = i / 149.0
                    interpolated = (1 - alpha) * current_joints + alpha * target_pos
                    ur5e.set_joint_targets(interpolated)
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
    print("=" * 70)
    print("UR5e + RH-P12-RN GRIPPER FREE CONTROL SIMULATION")
    print("=" * 70)
    
    if free_control_with_gripper():
        print("\n" + "=" * 70)
        print("✓ Free control simulation completed successfully!")
        print("=" * 70)
    else:
        print("\n" + "=" * 70)
        print("✗ Simulation failed. Please check the error messages above.")
        print("=" * 70)
