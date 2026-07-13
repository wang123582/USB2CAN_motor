#!/usr/bin/env python3
"""一键启动：底层驱动 + Delta 机械臂管理。

  ros2 launch motor_control_ros2 serve_bringup.launch.py

起来后手动发命令：
  ros2 topic pub --once /delta_arm/target motor_control_ros2/msg/ArmTarget \
    '{target_angles: [1.5, 1.5, 1.5], execute: true}'
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 底层硬件驱动（GO8010 串口 + DJI CAN）
        Node(package='motor_control_ros2', executable='motor_control_node',
             name='motor_control_node', output='screen'),
        # Delta 机械臂管理（软着陆→上抛冲顶限位→收拍→俯仰→回摆）
        Node(package='motor_control_ros2', executable='delta_arm_manager_node',
             name='delta_arm_manager', output='screen'),
    ])
