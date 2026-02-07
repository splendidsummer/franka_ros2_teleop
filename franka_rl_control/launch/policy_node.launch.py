from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='franka_rl_control',
            executable='rl_policy_node',
            name='rl_policy_node',
            output='screen',
            parameters=[
                {"end_effector_position": [0.3, 0.0, 0.5], 
                "end_effector_orientation": [0.0, 0.0, 0.0, 1.0]}
            ],
            remappings=[('/input_topic', '/franka_robot_state')]  # Adjust topic names as necessary
        ),
    ])