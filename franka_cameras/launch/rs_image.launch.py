import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Include the RealSense camera launch, disabling V4L2
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={'enable_v4l2': 'false'}.items()
    )

    # 2) Launch your subscriber node
    rs_listener = Node(
        package='franka_cameras',
        executable='rs_image',
        name='realsense_image_listener',
        output='screen',
        # remappings if needed:
        # remappings=[('/camera/color/image_raw','/my_cam/color')]
    )

    return LaunchDescription([
        realsense_launch,
        rs_listener,
    ])