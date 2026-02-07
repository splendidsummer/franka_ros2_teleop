import launch
import launch_ros.actions


def generate_launch_description():
    # 指定真正的手柄设备文件
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',     # ← 这里指定打算使用的 joystick 设备
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    pose_publisher = launch_ros.actions.Node(
        package='franka_joystick',
        executable='joystick_pose_publisher',
        name='joystick_pose_publisher',
        output='screen'
    )

    return launch.LaunchDescription([
        joy_node,
        pose_publisher,
    ])