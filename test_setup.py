'''
Test script for UR5e simulation setup
This script tests the basic functionality without requiring MuJoCo to be fully set up
'''

import os
import sys
import numpy as np

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def test_imports():
    """Test if all modules can be imported"""
    print("Testing imports...")
    
    try:
        from src import util
        print("✓ util module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import util: {e}")
        return False
    
    try:
        from src.PID import PID_ControllerClass
        print("✓ PID module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import PID: {e}")
        return False
    
    try:
        from src.ik_module import r2w, trim_scale
        print("✓ ik_module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import ik_module: {e}")
        return False
    
    # Test MuJoCo-dependent imports (might fail if MuJoCo not installed)
    try:
        from src.ur5e_simulator import UR5eSimulator
        print("✓ ur5e_simulator imported successfully")
    except ImportError as e:
        print(f"⚠ ur5e_simulator import failed (expected if MuJoCo not installed): {e}")
    
    return True

def test_utility_functions():
    """Test utility functions"""
    print("\nTesting utility functions...")
    
    from src.util import rpy2r, r2rpy, deg2rad, rad2deg, trim_scale
    
    # Test angle conversions
    angle_deg = 45.0
    angle_rad = deg2rad(angle_deg)
    angle_deg_back = rad2deg(angle_rad)
    
    if abs(angle_deg - angle_deg_back) < 1e-10:
        print("✓ Angle conversion functions work correctly")
    else:
        print("✗ Angle conversion functions failed")
        return False
    
    # Test rotation matrix functions
    rpy = np.array([0.1, 0.2, 0.3])
    R = rpy2r(rpy)
    rpy_back = r2rpy(R)
    
    if np.allclose(rpy, rpy_back):
        print("✓ Rotation matrix functions work correctly")
    else:
        print("✗ Rotation matrix functions failed")
        return False
    
    # Test trim_scale function
    x = np.array([1.0, 2.0, 3.0])
    x_trimmed = trim_scale(x, 2.0)
    expected = x * 2.0 / 3.0  # Should scale down since max is 3.0
    
    if np.allclose(x_trimmed, expected):
        print("✓ trim_scale function works correctly")
    else:
        print("✗ trim_scale function failed")
        return False
    
    return True

def test_pid_controller():
    """Test PID controller"""
    print("\nTesting PID controller...")
    
    from src.PID import PID_ControllerClass
    
    # Create PID controller
    pid = PID_ControllerClass(
        name='test_pid',
        k_p=1.0,
        k_i=0.1,
        k_d=0.01,
        dim=1
    )
    
    # Test basic functionality
    target = 1.0
    current = 0.0
    
    output = pid.get_output(x_trgt=target, x_curr=current, t_curr=0.1)
    
    if output is not None and len(output) == 1:
        print("✓ PID controller works correctly")
        print(f"  PID output: {output[0]:.4f}")
    else:
        print("✗ PID controller failed")
        return False
    
    return True

def test_file_structure():
    """Test if all required files exist"""
    print("\nTesting file structure...")
    
    required_files = [
        'ur5e_main.xml',
        'assets/ur5e/ur5e_assets.xml',
        'assets/ur5e/ur5e_body.xml',
        'assets/ur5e/ur5e_actuators.xml',
        'assets/common_arena/simple_plane.xml',
        'src/ur5e_simulator.py',
        'src/util.py',
        'src/PID.py',
        'src/ik_module.py',
        'src/__init__.py',
        'demo_ur5e.py'
    ]
    
    missing_files = []
    for file_path in required_files:
        full_path = os.path.join(os.path.dirname(__file__), file_path)
        if not os.path.exists(full_path):
            missing_files.append(file_path)
    
    if missing_files:
        print("✗ Missing files:")
        for file_path in missing_files:
            print(f"  - {file_path}")
        return False
    else:
        print("✓ All required files exist")
        return True

def main():
    """Run all tests"""
    print("=" * 60)
    print("UR5e Simulation Setup Test")
    print("=" * 60)
    
    all_passed = True
    
    # Test file structure
    if not test_file_structure():
        all_passed = False
    
    # Test imports
    if not test_imports():
        all_passed = False
    
    # Test utility functions
    if not test_utility_functions():
        all_passed = False
    
    # Test PID controller
    if not test_pid_controller():
        all_passed = False
    
    print("\n" + "=" * 60)
    if all_passed:
        print("✓ All tests passed! Your UR5e simulation setup is ready.")
        print("\nNext steps:")
        print("1. Install MuJoCo if not already installed")
        print("2. Run 'python demo_ur5e.py' to test the simulation")
        print("3. Modify the XML files as needed for your specific requirements")
    else:
        print("✗ Some tests failed. Please check the issues above.")
    print("=" * 60)

if __name__ == '__main__':
    main()
