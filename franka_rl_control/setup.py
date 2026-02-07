from setuptools import setup

setup(
    name='franka_rl_control',
    version='0.0.1',
    packages=['franka_rl_control'],
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'torch',
        'std_msgs',
        'franka_msgs',
        'pathlib'
    ],
    zip_safe=True,
    maintainer='Summer Zhang',
    maintainer_email='lzuzhangshsh@gmail.com',
    description='RL policy node for Franka ROS 2 control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rl_policy_node = franka_rl_control.rl_policy_node:main',
        ],
    },
)

