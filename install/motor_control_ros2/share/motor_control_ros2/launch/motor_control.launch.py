from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    ROS2 电机控制启动文件
    
    启动：
    1. C++ 实时控制节点（motor_control_node）
    2. Python 生命周期节点（lifecycle_node）
    """
    
    pkg_dir = get_package_share_directory('motor_control_ros2')
    
    # 启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'motors.yaml']),
        description='电机配置文件路径'
    )
    
    control_params_arg = DeclareLaunchArgument(
        'control_params',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'control_params.yaml']),
        description='控制参数文件路径'
    )
    
    # C++ 实时控制节点
    motor_control_node = Node(
        package='motor_control_ros2',
        executable='motor_control_node',
        name='motor_control_node',
        output='screen',
        parameters=[LaunchConfiguration('control_params')],
        emulate_tty=True
    )
    
    # Python 生命周期节点
    lifecycle_node = LifecycleNode(
        package='motor_control_ros2',
        executable='lifecycle_node',
        name='motor_lifecycle_node',
        namespace='',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')}
        ],
        emulate_tty=True
    )
    
    # 自动配置生命周期节点
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # 配置成功后自动激活
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    return LaunchDescription([
        config_file_arg,
        control_params_arg,
        motor_control_node,
        lifecycle_node,
        configure_event,
        activate_event,
    ])
