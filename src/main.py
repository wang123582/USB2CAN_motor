import time
from can_driver import CANDriver
from dji_motor import GM6020
from pid import PID

# === 配置 ===
MOTOR_ID = 1          # 假设是 ID 为 1 的 GM6020
TARGET_SPEED = 60.0   # 目标转速 RPM

# === 初始化 ===
driver = CANDriver()
motor = GM6020(MOTOR_ID)

# PID 参数调整建议 (GM6020 电压模式响应快)
# Kp: 提供主要动力
# Ki: 消除稳态误差 (必须有)
# Kd: 抑制振荡
pid = PID(kp=30.0, ki=0.0, kd=0.0, out_max=30000)

# === 接收回调 ===
def rx_handler(can_id, data):
    if can_id == motor.feedback_id:
        motor.parse_feedback(data)

driver.set_rx_callback(rx_handler)

# === 主循环 ===
try:
    print(f"开始控制 GM6020 (ID: {MOTOR_ID})")
    print(f"监听 ID: {hex(motor.feedback_id)}, 发送 ID: {hex(motor.control_id)}")
    
    while True:
        # 1. PID 计算
        output = pid.update(TARGET_SPEED, motor.rpm)
        
        # 2. 拼装 0x1FF 数据包
        # GM6020 ID 1 占用前两个字节 (Index 0, 1)
        # ID 2 占用 (Index 2, 3) ...
        can_payload = [0] * 8
        cmd_bytes = motor.get_voltage_bytes(output)
        
        idx = (MOTOR_ID - 1) * 2
        if idx < 7:
            can_payload[idx] = cmd_bytes[0]
            can_payload[idx+1] = cmd_bytes[1]
            
        # 3. 发送控制帧
        driver.send_can_frame(motor.control_id, can_payload)
        
        # 4. 打印状态
        print(f"Tgt: {TARGET_SPEED:4.0f} | Real: {motor.rpm:4d} | Out: {int(output):6d} | Temp: {motor.temp}°C")
        
        time.sleep(0.005) # 200Hz 控制频率

except KeyboardInterrupt:
    print("停止...")
    # 发送 0 电压停止电机
    driver.send_can_frame(motor.control_id, [0]*8)
    driver.running = False