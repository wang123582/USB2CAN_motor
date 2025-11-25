import time
from can_driver import CANDriver
from multi_motor_manager import MultiMotorManager

# === 配置多电机 ===
MOTOR_CONFIGS = [
    # 格式: (motor_id, target_angle, speed_pid, angle_pid)
    {
        'id': 1,
        'target_angle': 90.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 300, 'out_max': 10000, 'dead_zone': 5},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 10, 'out_max': 200, 'dead_zone': 0.5}
    },
    {
        'id': 2,
        'target_angle': 180.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 300, 'out_max': 10000, 'dead_zone': 5},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_max': 10, 'out_max': 200, 'dead_zone': 0.5}
    },
    # 添加更多电机配置...
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
    while True:
        # 发送所有电机的控制命令
        manager.send_commands()
        
        # 每50次循环打印一次状态 (约0.25秒)
        if loop_count % 50 == 0:
            manager.print_status()
            print()  # 空行分隔
        
        loop_count += 1
        time.sleep(0.005)  # 200Hz

except KeyboardInterrupt:
    print("\n停止所有电机...")
    manager.stop_all()
    driver.running = False
    print("程序结束")