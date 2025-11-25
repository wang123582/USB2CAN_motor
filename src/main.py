import time
import sys
import signal
from can_driver import CANDriver
from multi_motor_manager import MultiMotorManager

# === 全局变量用于信号处理 ===
shutdown_flag = False

def signal_handler(sig, frame):
    """处理 Ctrl+C 信号"""
    global shutdown_flag
    print("\n\n🛑 收到退出信号 (Ctrl+C)...")
    shutdown_flag = True

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)

# === ROS集成 (可选) ===
USE_ROS = '--ros' in sys.argv

if USE_ROS:
    try:
        import rospy
        import ros_angle_updater
        
        # 初始化ROS
        try:
            ros_angle_updater.init_ros_node()
        except rospy.exceptions.ROSException as e:
            print(f"\n❌ ROS 初始化失败: {e}")
            print("\n💡 解决方法:")
            print("   1. 先启动 roscore:")
            print("      roscore")
            print("   或")
            print("   2. 使用一键启动脚本:")
            print("      bash start_ros_system.sh")
            print("\n")
            sys.exit(1)
        
        # 定义更新回调
        def on_angle_update(motor_id, new_angle):
            """当ROS话题更新角度时触发"""
            if motor_id in manager.motors:
                manager.set_target_angle(motor_id, new_angle)
                print(f"[ROS更新] 电机{motor_id} -> {new_angle}°")
        
        # 注册回调
        ros_angle_updater.register_callback(on_angle_update)
        
        print("[主程序] ROS模式已启用")
        print("[提示] 按 Ctrl+C 可随时退出\n")
        ROS_ENABLED = True
        
    except ImportError as e:
        print(f"\n❌ ROS模块导入失败: {e}")
        print("\n💡 解决方法:")
        print("   bash setup_conda_ros.sh")
        print("   conda deactivate && conda activate base")
        print("\n")
        sys.exit(1)
    except Exception as e:
        print(f"[警告] ROS初始化失败: {e}")
        ROS_ENABLED = False
else:
    ROS_ENABLED = False
    print("[主程序] 非ROS模式")
    print("[提示] 按 Ctrl+C 可随时退出\n")

# === 配置多电机 ===
MOTOR_CONFIGS = [
    {
        'id': 1,
        'target_angle': 90.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 300, 'out_max': 5000, 'dead_zone': 5},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 10, 'out_max': 200, 'dead_zone': 0.5}
    },
    {
        'id': 2,
        'target_angle': 180.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 300, 'out_max': 5000, 'dead_zone': 5},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 10, 'out_max': 200, 'dead_zone': 0.5}
    },
]

# === 初始化 ===
driver = CANDriver()
manager = MultiMotorManager(driver)

# 添加所有配置的电机
for config in MOTOR_CONFIGS:
    manager.add_motor(
        motor_id=config['id'],
        target_angle=config['target_angle'],
        speed_pid_params=config['speed_pid'],
        angle_pid_params=config['angle_pid']
    )

# === 主控制循环 ===
try:
    print("开始多电机双环控制")
    print(f"控制电机数量: {len(MOTOR_CONFIGS)}")
    
    # 等待所有电机初始数据
    print("等待电机反馈...")
    time.sleep(0.5)
    print("开始闭环控制\n")

    loop_count = 0
    
    # 统一的循环条件：检查 shutdown_flag 和 ROS 状态
    while not shutdown_flag:
        # ROS模式下额外检查 rospy 是否被关闭
        if ROS_ENABLED and rospy.is_shutdown():
            break
        
        # 发送所有电机的控制命令
        manager.send_commands()
        
        # 每50次循环打印一次状态 (约0.25秒)
        if loop_count % 50 == 0:
            manager.print_status()
            print()
        
        loop_count += 1
        time.sleep(0.005)  # 200Hz

except KeyboardInterrupt:
    # 这个分支在某些情况下仍会触发（如非ROS模式）
    print("\n🛑 收到键盘中断...")
except Exception as e:
    print(f"\n❌ 错误: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("\n正在安全停止所有电机...")
    manager.stop_all()
    driver.running = False
    
    # ROS模式下主动关闭节点
    if ROS_ENABLED:
        try:
            rospy.signal_shutdown("用户请求退出")
        except:
            pass
    
    print("✅ 程序已安全退出")