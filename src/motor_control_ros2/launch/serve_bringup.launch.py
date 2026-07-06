#!/usr/bin/env python3
"""发球全链路一键启动：只拉起三个节点，命令自己手动发。

  ros2 launch motor_control_ros2 serve_bringup.launch.py

起来后自己手动：
  # 让 windmill 引拍到 WIND_UP 待命
  ros2 topic pub --once /serve/command std_msgs/msg/String '{data: start}'
  # 发一发球（delta 上抛→收拍→触发 windmill）
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
        # 风车发球机构
        Node(package='motor_control_ros2', executable='serve_windmill_manager',
             name='serve_windmill_manager', output='screen'),
        # Delta 机械臂管理（软着陆→上抛冲顶限位→收拍→触发 windmill→等接住完毕→俯仰回摆）
        Node(package='motor_control_ros2', executable='delta_arm_manager_node',
             name='delta_arm_manager', output='screen'),
    ])
