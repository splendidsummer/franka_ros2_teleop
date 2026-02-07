# Franka Gazebo

**==Important Note:==**

Minimum necessary `franka_description` version is 0.3.0.
You can clone franka_description package from https://github.com/frankaemika/franka_description.

A project integrating Franka ROS 2 with the Gazebo simulator.

## Launch RVIZ + Gazebo

Launch an example which spawns RVIZ and Gazebo showing the robot:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py
```

If you want to display another robot, you can define the arm_id:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py arm_id:=fp3
```

If you want to start the simulation including the franka_hand:

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py load_gripper:=true franka_hand:='franka_hand'
```


## Joint Velocity Control Example with Gazebo

Before starting, be sure to build `franka_example_controllers` and `franka_description` packages.
`franka_description` must have the minimum version of 0.3.0.

```bash
colcon build --packages-select franka_example_controllers
```

Now you can launch the velocity example with Gazebo simulator.

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_velocity_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

Keep in mind that the gripper joint has a bug with the joint velocity controller. 
If you are interested in controlling the gripper please use joint position interface.


## Joint Position Control Example with Gazebo

To run the joint position control example you need to have the required software listed in the joint velocity control section.

Then you can run with the following command.

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

## Joint Impedance Control Example with Gazebo

For running torque example. You must compile the `franka_ign_ros2_control` package located under `franka_gazebo`.
You can compile `franka_ign_ros2_control` with the following command.

```bash
colcon build --packages-select franka_ign_ros2_control
```

Then source your workspace.

```bash
source install/setup.sh
```

Then you can run the impedance control example.

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_impedance_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```


## Joystick Cartesian Pose Control Example with Gazebo

This example allows you to control the robot's end-effector pose using a game controller/joystick.

```bash
ros2 launch franka_gazebo_bringup gazebo_joystick_cartesian_pose_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

**Requirements:**
- A compatible game controller/joystick connected to your system
- The controller should be properly configured in the system

**Note:** The joystick controller maps joystick axes to Cartesian position/orientation commands. Refer to the controller implementation for specific axis mappings.

## Available Launch Files

| Launch File | Purpose | Controller Loaded |
|-------------|---------|-------------------|
| `visualize_franka_robot.launch.py` | Visualization only (Gazebo + RViz) | `joint_state_broadcaster` |
| `gazebo_joint_velocity_controller_example.launch.py` | Joint velocity control | `joint_velocity_example_controller` |
| `gazebo_joint_position_controller_example.launch.py` | Joint position control | `joint_position_example_controller` |
| `gazebo_joint_impedance_controller_example.launch.py` | Joint impedance (torque) control | `joint_impedance_example_controller` |
| `gazebo_joystick_cartesian_pose_controller_example.launch.py` | Joystick teleoperation | `joystick_cartesian_pose_controller` |

## Common Launch Arguments

All launch files support the following arguments:

| Argument | Default Value | Description |
|----------|---------------|-------------|
| `arm_id` | `fr3` | Robot model (fr3, fp3, fer) |
| `load_gripper` | `false` | Enable/disable gripper |
| `franka_hand` | `franka_hand` | Gripper type |
| `namespace` | `""` | Robot namespace (for multi-robot setups) |

Example with custom arguments:
```bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py \
  arm_id:=fr3 \
  load_gripper:=true \
  franka_hand:=franka_hand \
  namespace:=robot1
```

## Configuration Notes

The Gazebo simulation uses a separate controller configuration file located at:
```
franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
```

Key configuration details:
- **Controller types**: Defined for joint position, joint velocity, joint impedance, and joystick cartesian pose control
- **Gazebo flag**: All controllers have `gazebo: true` parameter to enable Gazebo-specific behavior
- **Impedance control gains**: Pre-configured stiffness (k_gains) and damping (d_gains) parameters
- **Update rate**: Controller manager runs at 1000Hz for simulation

To modify controller parameters, edit the YAML file and rebuild the workspace:
```bash
colcon build --packages-select franka_gazebo_bringup
```

## Troubleshooting

If you experience that Gazebo can't find your model files, try to include the workspace. E.g.

```bash
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/workspaces/src/
```
