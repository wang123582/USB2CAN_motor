from setuptools import setup
from glob import glob
import os

package_name = 'motor_control_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 电机控制包 - 支持 DJI、达妙、宇树电机',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_node = motor_control_ros2.lifecycle_node:main',
        ],
    },
)
