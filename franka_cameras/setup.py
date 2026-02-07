from setuptools import setup
from glob import glob
import os

package_name = 'franka_cameras'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # install resource marker
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        # install launch files
        ('share/'+package_name+'/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',     # for cv2
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'image_transport',
    ],
    zip_safe=True,
    maintainer='Summer Zhang',
    maintainer_email='lzuzhangshsh@gmail.com',
    description='RealSense image listener for Franka',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # this makes `ros2 run franka_cameras rs_image` work
            'rs_image = franka_cameras.rs_image:main',
        ],
    },
)