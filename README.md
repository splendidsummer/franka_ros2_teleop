# Franka ROS2 Workspace

A comprehensive ROS 2 Humble workspace for Franka Emika research robots, providing hardware interfaces, controllers, simulation, and research tools for the Franka FR3 robot.

## Overview

This workspace integrates Franka robots with ROS 2 through the `libfranka` library, offering a complete ecosystem for robotic research and development. The system supports real robot control, simulation, multiple control strategies, and research extensions like reinforcement learning and computer vision.

## Architecture

### Core Components

The workspace follows a modular ROS 2 architecture built on `ros2_control`:

```
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                      │
│  (Example Controllers, Research Packages, MoveIt2)      │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                    Control Layer                         │
│          (ROS 2 Control Framework - controller_manager) │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                  Hardware Abstraction                    │
│      (franka_hardware - SystemInterface implementation) │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                   Communication Layer                    │
│                (libfranka - FCI protocol)               │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                    Physical/SIM Layer                    │
│          (Real Franka Robot / Gazebo Simulation)        │
└─────────────────────────────────────────────────────────┘
```

### Key Architectural Features

- **Namespace Support**: Full namespace isolation for multi-robot setups
- **Hardware Abstraction**: Unified interface for real hardware and simulation
- **Plugin-based Controllers**: Extensible controller architecture via `pluginlib`
- **Semantic Components**: Typed interfaces for robot state and commands
- **Real-time Capable**: Designed for real-time control with proper kernel configuration

## Packages

### Core Infrastructure

| Package | Purpose | Key Components |
|---------|---------|----------------|
| `franka_hardware` | Hardware interface implementation | `FrankaHardwareInterface`, `Robot`, `FrankaExecutor` |
| `franka_msgs` | Custom messages, services, actions | `FrankaRobotState`, collision services, grasp actions |
| `franka_semantic_components` | Semantic interfaces for ROS 2 Control | `FrankaRobotState` component |

### Control System

| Package | Purpose | Features |
|---------|---------|----------|
| `franka_example_controllers` | Example control implementations | 15+ controllers for joint/cartesian space control |
| `franka_robot_state_broadcaster` | State information publisher | Configurable lock management |
| `franka_bringup` | Launch files and configuration | Namespace support, multi-robot, fake hardware |

### Simulation

| Package | Purpose | Features |
|---------|---------|----------|
| `franka_gazebo` | Gazebo/Ingition integration | Joint position/velocity/impedance control examples |
| `franka_ign_ros2_control` | Ignition Gazebo ROS 2 Control plugin | Torque control support |

### Robot Description & Planning

| Package | Purpose | Features |
|---------|---------|----------|
| `franka_description` | URDF models and meshes | FR3 robot description |
| `franka_fr3_moveit_config` | MoveIt2 configuration | OMPL planners, kinematics, visualization |
| `franka_gripper` | Gripper control | Python/C++ implementations |

### Research & Extensions

| Package | Purpose | Features |
|---------|---------|----------|
| `franka_rl_control` | Reinforcement learning integration | PyTorch policy execution |
| `franka_cameras` | RealSense camera integration | Image transport, CV bridge |
| `franka_sideview_cameras` | Dual camera publisher | PyRealSense2, OpenCV |
| `franka_joystick` | Teleoperation interface | Game controller to Cartesian pose |
| `franka_vla_inference_service` | Vision-language-action inference | (In development) |

## Available Controllers

The system provides a comprehensive set of controllers for different control strategies:

### Joint Space Control
- **Joint Position Control**: Position-based joint control
- **Joint Velocity Control**: Velocity-based joint control
- **Joint Impedance Control**: Torque-based impedance control
- **Joint Impedance with IK**: Impedance control with inverse kinematics

### Cartesian Space Control
- **Cartesian Pose Control**: End-effector position/orientation control
- **Cartesian Velocity Control**: End-effector velocity control
- **Cartesian Elbow Control**: Cartesian control with elbow configuration
- **Cartesian Orientation Control**: Orientation-only control

### Specialized Controllers
- **Gravity Compensation**: Zero-force gravity compensation
- **Move to Start**: Smooth movement to start position
- **Model-Based Control**: Model-based torque control
- **Gripper Control**: Parallel gripper control
- **Dynamic Wave**: Wave pattern demonstration
- **Joystick Cartesian Pose**: Teleoperation controller

## Getting Started

### Prerequisites
- **ROS 2 Humble** (Desktop or Base installation)
- **Real-time kernel** (for real robot operation)
- **Docker** (optional, for containerized development)

### Installation

```bash
# Create workspace
mkdir -p ~/franka_ros2_ws/src
cd ~/franka_ros2_ws

# Clone repository
git clone https://github.com/frankaemika/franka_ros2.git src

# Import dependencies
vcs import src < src/franka.repos --recursive --skip-existing

# Install ROS dependencies
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

### Quick Start Examples

#### Simulation (No Robot Required)
```bash
# Launch MoveIt2 with fake hardware
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# Run a specific controller
ros2 launch franka_bringup example.launch.py controller_name:=joint_position_example_controller
```

#### Gazebo Simulation
```bash
# Visualize robot in Gazebo
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py

# Joint position control example
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py
```

#### Real Robot Operation
```bash
# Configure robot IP in franka.config.yaml first
ros2 launch franka_bringup franka.launch.py
```

## Configuration

### Robot Configuration (`franka_bringup/config/franka.config.yaml`)
```yaml
franka:
  ros__parameters:
    robot_ip: "10.0.0.2"  # Robot IP address
    namespace: ""         # Robot namespace
    use_fake_hardware: false  # Use simulation
    control_mode: "position"  # Default control mode
```

### Controller Configuration (`franka_bringup/config/controllers.yaml`)
- Namespace-agnostic by default (uses `/**` wildcard)
- Supports per-namespace parameter overrides
- Configures controller types and parameters

### Namespace Support
The system supports multiple robots through namespaces:
```yaml
# Global default
/**:
  franka_robot_state_broadcaster:
    ros__parameters:
      lock_try_count: 5

# Namespace-specific override
/robot1:
  franka_robot_state_broadcaster:
    ros__parameters:
      lock_try_count: 10
```

## Development

### Creating Custom Controllers
1. Extend `controller_interface::ControllerInterface`
2. Implement required lifecycle methods
3. Register as plugin in `CMakeLists.txt`
4. Add to `controllers.yaml` configuration

Example controller structure:
```cpp
class MyCustomController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update() override;
};
```

### Hardware Interface Extension
The `FrankaHardwareInterface` implements `hardware_interface::SystemInterface` and provides:
- Command interfaces: effort, position, velocity, cartesian commands
- State interfaces: joint positions, velocities, robot state
- Mode switching: Dynamic control mode transitions

### Research Integration
- **RL Policies**: Use `franka_rl_control` as template for policy execution
- **Perception**: Integrate cameras via `franka_cameras` packages
- **Teleoperation**: Extend `franka_joystick` for custom teleop interfaces

## Testing

### Unit Tests
```bash
# Run tests for specific package
colcon test --packages-select franka_hardware

# View test results
colcon test-result --verbose
```

### Integration Tests
```bash
# Run integration tests
colcon test --packages-select integration_launch_testing
```

### Simulation Testing
- Use `use_fake_hardware:=true` for hardware-free testing
- Gazebo provides physics-based simulation
- Consistent interface between simulation and real hardware

## Troubleshooting

### Common Issues

1. **UDP Timeout Errors**
   - Ensure real-time kernel is installed
   - Avoid Docker Desktop (use Docker Engine)
   - Check network connectivity to robot

2. **Controller Not Found**
   - Verify controller name in `controllers.yaml`
   - Check package is built: `colcon list --packages-select`
   - Ensure workspace is sourced

3. **Namespace Issues**
   - Verify namespace configuration in launch files
   - Check `controllers.yaml` for namespace-specific configurations
   - Ensure topic names include namespace prefix

4. **Build Failures**
   - Run `rosdep install` to ensure dependencies
   - Check `franka.repos` for external dependencies
   - Verify ROS 2 Humble is installed

### Real-time Configuration
For real robot operation:
```bash
# Install real-time kernel
sudo apt-get install linux-image-rt

# Configure Docker for real-time (if using containers)
--cap-add=SYS_NICE --ulimit rtprio=99
```

## Research Applications

### Reinforcement Learning
- `franka_rl_control` provides ROS 2 interface for policy execution
- Subscribe to `FrankaRobotState`, publish `rl_policy_action`
- PyTorch integration for model loading and inference

### Computer Vision
- RealSense camera integration via `franka_cameras`
- Dual camera support with `franka_sideview_cameras`
- OpenCV and ROS 2 image transport

### Motion Planning
- MoveIt2 integration for motion planning
- OMPL planner configuration
- RViz visualization and planning interface

## Contributing

1. Follow ROS 2 coding standards
2. Use `clang-format` and `clang-tidy` for code quality
3. Add unit tests for new functionality
4. Update `controllers.yaml` for new controllers
5. Document new features in appropriate README files

### Commit Sign-off
All contributions require DCO sign-off:
```
Signed-off-by: Your Name <your.email@example.com>
```

## License

All packages are licensed under Apache 2.0. See `LICENSE` files for details.

## Resources

- [Official Documentation](https://frankaemika.github.io/docs)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [Issue Tracker](https://github.com/frankaemika/franka_ros2/issues)

## Support

- GitHub Issues for bug reports and feature requests
- ROS Answers for usage questions
- Franka Community for robot-specific questions