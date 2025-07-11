'''
Simple XML validation test for UR5e simulation
'''

import os
import sys

def test_xml_loading():
    """Test XML loading without MuJoCo viewer"""
    try:
        import mujoco
        print("Testing XML loading...")
        
        # Try to load the main XML file
        xml_path = os.path.join(os.path.dirname(__file__), 'ur5e_main.xml')
        print(f"Loading XML: {xml_path}")
        
        model = mujoco.MjModel.from_xml_path(xml_path)
        print("✓ XML loaded successfully!")
        print(f"  Number of bodies: {model.nbody}")
        print(f"  Number of joints: {model.njnt}")
        print(f"  Number of actuators: {model.nu}")
        
        return True
        
    except ImportError:
        print("MuJoCo not installed - cannot test XML loading")
        return False
    except Exception as e:
        print(f"✗ XML loading failed: {e}")
        return False

if __name__ == '__main__':
    test_xml_loading()
