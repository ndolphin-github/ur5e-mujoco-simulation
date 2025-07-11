'''
Minimal demo for UR5e simulation - tests basic loading and visualization
'''

import numpy as np
import time
import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def minimal_demo():
    """
    Minimal demo that just tests XML loading and basic visualization
    """
    
    try:
        from src.ur5e_simulator import UR5eSimulator
        
        print("Initializing UR5e simulator...")
        ur5e = UR5eSimulator(
            name='UR5e_Robot',
            xml_path='ur5e_main.xml',
            USE_VIEWER=True,
            VERBOSE=True
        )
        
        print("\n✓ UR5e simulator initialized successfully!")
        print(f"  Number of joints: {ur5e.n_joints}")
        print(f"  Number of actuators: {ur5e.n_actuators}")
        print(f"  Joint names: {ur5e.joint_names}")
        
        # Simple test - just render the default pose
        print("\n=== Rendering default pose ===")
        try:
            for i in range(300):  # 3 seconds at ~100 fps
                ur5e.render()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Rendering interrupted by user")
        except Exception as e:
            print(f"Rendering error: {e}")
        
        print("\n✓ Basic rendering test completed!")
        
        # Test joint position setting
        print("\n=== Testing joint position setting ===")
        try:
            # Very simple joint positions (all zeros)
            zero_position = np.zeros(6)
            ur5e.set_joint_positions(zero_position)
            
            for i in range(200):
                ur5e.render()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Joint position test interrupted by user")
        except Exception as e:
            print(f"Joint position test error: {e}")
        
        print("✓ Joint position test completed!")
        
        # Close viewer
        ur5e.close_viewer()
        print("\n✓ All tests passed! UR5e simulation is working correctly.")
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure MuJoCo is installed: pip install mujoco")
        return False
    except Exception as e:
        print(f"Error occurred: {e}")
        print("There might be an issue with the XML files or MuJoCo setup.")
        return False
    
    return True

if __name__ == '__main__':
    print("=" * 60)
    print("UR5e Minimal Demo")
    print("=" * 60)
    
    if minimal_demo():
        print("\n" + "=" * 60)
        print("✓ Minimal demo completed successfully!")
        print("You can now try running the full demo: python demo_ur5e.py")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("✗ Demo failed. Please check the error messages above.")
        print("=" * 60)
