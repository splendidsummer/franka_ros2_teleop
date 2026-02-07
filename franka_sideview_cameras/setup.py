from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'franka_sideview_cameras'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], 
    data_files=[
        # 安装 package.xml 和资源标记
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        # # 安装 launch 文件
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'opencv-python',
        'pyrealsense2',
    ],
    zip_safe=True,
    maintainer='summer',
    maintainer_email='summer@todo.todo',
    description='Dual RealSense camera publisher node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dual_camera_publisher = franka_sideview_cameras.dual_camera_publisher:main',
        ],
    },
)
