# UR5e MuJoCo Simulation

A comprehensive simulation setup for the Universal Robots UR5e robot using MuJoCo physics engine.

## Project Structure

```
ur5e_simulation/
├── ur5e_main.xml              # Main simulation XML file
├── assets/                    # Asset files
│   ├── ur5e/                 # UR5e robot assets
│   │   ├── ur5e_assets.xml   # Materials and mesh definitions
│   │   ├── ur5e_body.xml     # Robot body structure
│   │   ├── ur5e_actuators.xml # Actuator definitions
│   │   └── meshes/           # STL mesh files
│   └── common_arena/         # Environment assets
│       └── simple_plane.xml  # Ground plane
├── src/                      # Source code
│   ├── __init__.py          # Package initialization
│   ├── ur5e_simulator.py    # Main simulator class
│   ├── util.py              # Utility functions
│   ├── PID.py               # PID controller
│   └── ik_module.py         # Inverse kinematics utilities
├── demo_ur5e.py             # Demonstration script
├── test_setup.py            # Setup validation script
└── README.md                # This file
```

## Features

### Core Functionality
- **Complete UR5e Model**: Accurate 6-DOF UR5e robot with proper joint limits and dynamics
- **MuJoCo Integration**: Full integration with MuJoCo physics engine
- **Interactive Viewer**: Real-time 3D visualization with camera controls
- **Forward Kinematics**: Compute end-effector pose from joint angles
- **Inverse Kinematics**: Solve for joint angles given target end-effector pose
- **Joint Control**: Position and velocity control of individual joints

### Simulation Components
- **Physics Simulation**: High-fidelity physics simulation with collision detection
- **Visual Rendering**: Real-time rendering with customizable materials and lighting
- **Trajectory Planning**: Tools for planning and executing robot trajectories
- **PID Control**: Built-in PID controller for joint position control

## Installation

### Prerequisites
1. **Python 3.7+**
2. **MuJoCo**: Install MuJoCo physics engine
   ```bash
   pip install mujoco
   ```
3. **NumPy**: For numerical computations
   ```bash
   pip install numpy
   ```

### Setup
1. Clone or download this project
2. Ensure all mesh files are in the correct location (`assets/ur5e/meshes/`)
3. Run the setup test:
   ```bash
   python test_setup.py
   ```

## Usage

### Basic Usage

```python
from src.ur5e_simulator import UR5eSimulator
import numpy as np

# Initialize simulator
ur5e = UR5eSimulator(
    xml_path='ur5e_main.xml',
    USE_VIEWER=True
)

# Set joint positions (6 joints in radians)
home_position = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
ur5e.set_joint_positions(home_position)

# Get end-effector pose
position, rotation = ur5e.get_end_effector_pose()

# Render simulation
for i in range(1000):
    ur5e.render()
```

### Inverse Kinematics

```python
# Define target position
target_pos = np.array([0.3, 0.3, 0.5])

# Solve IK
joint_solution = ur5e.inverse_kinematics(target_pos)

if joint_solution is not None:
    ur5e.set_joint_positions(joint_solution)
```

### Running Demos

1. **Setup Test**: Validate installation
   ```bash
   python test_setup.py
   ```

2. **Basic Demo**: Run comprehensive demonstration
   ```bash
   python demo_ur5e.py
   ```

## XML Configuration

### Main Configuration (`ur5e_main.xml`)
- Simulation parameters (timestep, solver settings)
- Include statements for sub-components
- Default joint and material properties
- Keyframe definitions for common poses

### Robot Definition (`assets/ur5e/`)
- **ur5e_assets.xml**: Material properties and mesh references
- **ur5e_body.xml**: Robot link structure and joint definitions
- **ur5e_actuators.xml**: Motor and control definitions

### Environment (`assets/common_arena/`)
- Ground plane and environmental objects
- Lighting and camera definitions

## Customization

### Adding New End-Effectors
1. Add mesh files to `assets/ur5e/meshes/`
2. Update `ur5e_body.xml` with new link definitions
3. Modify `end_effector_name` in simulator class

### Custom Control Strategies
```python
# Example: Custom PID control
from src.PID import PID_ControllerClass

pid_controller = PID_ControllerClass(
    k_p=10.0, k_i=0.1, k_d=1.0, dim=6
)

# In control loop
target_joints = np.array([...])  # Target joint positions
current_joints = ur5e.get_joint_positions()
control_output = pid_controller.get_output(
    x_trgt=target_joints, 
    x_curr=current_joints, 
    t_curr=ur5e.get_simulation_time()
)
ur5e.step(ctrl=control_output)
```

### Adding Sensors
Modify XML files to include:
- Force/torque sensors
- IMU sensors  
- Camera sensors
- Proximity sensors

## API Reference

### UR5eSimulator Class

#### Core Methods
- `__init__(xml_path, USE_VIEWER=True)`: Initialize simulator
- `reset()`: Reset simulation to initial state
- `step(ctrl=None, nstep=1)`: Step simulation forward
- `render()`: Update visualization
- `close_viewer()`: Close simulation window

#### Kinematics
- `get_joint_positions()`: Get current joint angles
- `set_joint_positions(positions)`: Set joint angles
- `get_end_effector_pose()`: Get end-effector position and orientation
- `get_jacobian(body_name)`: Get Jacobian matrix
- `inverse_kinematics(target_pos, target_rot=None)`: Solve IK

#### Utilities
- `add_visual_marker(position, size, color)`: Add visual markers
- `get_simulation_time()`: Get current simulation time
- `is_viewer_alive()`: Check if viewer is active

## Troubleshooting

### Common Issues

1. **ImportError: No module named 'mujoco'**
   - Install MuJoCo: `pip install mujoco`

2. **XML parsing errors**
   - Check that all mesh files exist in `assets/ur5e/meshes/`
   - Verify XML file paths and syntax

3. **Viewer not opening**
   - Check display settings
   - Try running with `USE_VIEWER=False` for headless mode

4. **IK not converging**
   - Target position may be outside workspace
   - Adjust IK parameters (tolerance, max_iterations, step_size)
   - Check for joint limit violations

### Performance Tips
- Use smaller timesteps for more accurate simulation
- Disable viewer for faster computation
- Use appropriate solver settings for your application

## Contributing

Feel free to contribute by:
- Adding new robot models
- Implementing advanced control algorithms
- Improving IK solvers
- Adding more comprehensive examples

## License

This project is based on components from Simple-MuJoCo-PickNPlace and MuJoCo tutorials.

## References
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Universal Robots UR5e](https://www.universal-robots.com/products/ur5-robot/)
- [Simple-MuJoCo-PickNPlace](https://github.com/sjchoi86/Simple-MuJoCo-PickNPlace)
