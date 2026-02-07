# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **ROS 2 Humble** workspace for **Franka Emika Robotics Research Robots** (`franka_ros2`). The repository provides ROS 2 integration of `libfranka` for controlling Franka research robots. The workspace follows standard ROS 2 structure with packages for hardware interfaces, controllers, simulation, and research applications.

## Build System

- **Primary build tool**: `colcon` (ROS 2 standard)
- **Standard build command**: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- **CMake-based packages**: All packages use `ament_cmake`
- **Dependency management**: `rosdep` and `vcs` (via `franka.repos` file)

## Common Development Commands

### Building
```bash
# Standard build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build specific package
colcon build --packages-select <package_name>

# Build with debug symbols
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing
```bash
# Run all tests
colcon test

# View test results
colcon test-result --verbose

# Run tests for specific package
colcon test --packages-select <package_name>
```

### Environment Setup
```bash
# Source the workspace
source install/setup.bash

# Source ROS 2 base (if needed)
source /opt/ros/humble/setup.bash
```

### Running Applications
```bash
# Test without robot (simulation)
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# Run example controller (configure in franka.config.yaml first)
ros2 launch franka_bringup example.launch.py controller_name:=your_desired_controller

# Run with namespace support
ros2 launch franka_bringup example.launch.py namespace:=your_namespace
```

### Dependency Management
```bash
# Update external dependencies
vcs import src < src/franka.repos --recursive --skip-existing

# Install ROS dependencies
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

## Architecture Overview

### Package Structure
- **Core**: `franka_ros2` (meta), `franka_hardware`, `libfranka`, `franka_description`, `franka_msgs`
- **Control**: `franka_example_controllers`, `franka_robot_state_broadcaster`, `franka_semantic_components`
- **Application**: `franka_bringup`, `franka_fr3_moveit_config`, `franka_gripper`
- **Simulation**: `franka_gazebo`, `franka_ign_ros2_control`
- **Research**: `franka_cameras`, `franka_joystick`, `franka_rl_control`, `franka_vla_inference_service`

### Key Architectural Patterns

1. **Namespace Support**: All packages support namespaced operation via configuration in `franka.config.yaml`
2. **Hardware Abstraction**: `franka_hardware` provides interface for both real hardware and fake hardware simulation
3. **Controller Framework**: Built on ROS 2 Control with multiple example controllers (Cartesian, Joint, Impedance, etc.)
4. **Real-time Requirements**: Real-time kernel required for actual robot communication (not needed for simulation)

### Configuration Files
- `src/franka_bringup/config/franka.config.yaml` - Robot configuration
- `src/franka_bringup/config/controllers.yaml` - Controller definitions (namespace-agnostic)
- `src/franka.repos` - External dependencies (libfranka v0.15.0, franka_description v0.4.0)
- `.vscode/settings.json` - Development environment settings

## Development Environment

### Docker Support
The project includes complete Docker setup:
- `Dockerfile` - Base container with ROS 2 Humble and dependencies
- `docker-compose.yml` - Container orchestration
- `.devcontainer/devcontainer.json` - VSCode devcontainer configuration
- Real-time capabilities enabled (`privileged: true`, `cap_add: SYS_NICE`)

### VSCode Configuration
- ROS 2 extension (`ms-iot.vscode-ros`)
- Python/C++ development tools
- Clang-format and clang-tidy integration
- Python path configuration for ROS packages

### Code Quality
- **Clang-format** based on Chromium style
- **Clang-tidy** static analysis
- **Apache 2.0** license
- **Signed-off-by** requirement for commits (DCO compliance)

## Important Notes

1. **Real-time Kernel**: Required for actual robot communication to prevent UDP timeout errors
2. **Docker Desktop Limitation**: Avoid Docker Desktop; use Docker Engine for real-time capabilities
3. **Namespace Configuration**: Controllers are namespace-agnostic by default; modify `controllers.yaml` for namespace-specific configurations
4. **Simulation vs Real Hardware**: Use `use_fake_hardware:=true` for simulation/testing without robot
5. **Controller Selection**: Configure desired controller in `franka.config.yaml` before launching

## Troubleshooting

- **UDP timeout errors**: Ensure real-time kernel is installed and Docker Desktop is not used
- **Build failures**: Run `rosdep install` to ensure all dependencies are installed
- **Controller not found**: Verify controller name in `controllers.yaml` and configuration in `franka.config.yaml`
- **Namespace issues**: Check `franka.config.yaml` for proper namespace configuration

## Package Development

When creating new packages:
1. Follow ROS 2 package structure conventions
2. Use `ament_cmake` for C++ packages
3. Add dependencies to `package.xml`
4. Configure controllers in `controllers.yaml` if applicable
5. Test with both real hardware and fake hardware modes

## Testing Strategy
- Unit tests for individual components
- Integration tests with fake hardware
- Simulation tests in Gazebo
- Real hardware tests (requires robot connection)