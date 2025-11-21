import time
from can_driver import CANDriver
from dji_motor import DJIMotor
from pid import PIDController

# === 配置 ===
# 你的日志显示收到的是 0x205，这通常是 5号电机，或者是 GM6020 的 ID 1
# 为了匹配你的硬件，我们这里设为 5，并在下文特殊处理
MOTOR_ID = 5   
TARGET_SPEED = 100 # 目标 500 RPM

# === 初始化 ===
driver = CANDriver()
# 0x205 反馈ID = 0x200 + 5
motor = DJIMotor(MOTOR_ID) 

pid = PIDController(kp=50.0, ki=0.0, kd=0.1, out_min=-20000, out_max=20000)

# === 回调函数 ===
def rx_handler(can_id, data):
    # 过滤 ID，只处理我们关心的电机
    if can_id == motor.feedback_id:
        motor.parse_feedback(data)
        
        # 调试打印 (每 20次 打印一次，避免刷屏)
        # if int(time.time()*100) % 20 == 0:
        #    print(f"[Rx] ID: {hex(can_id)} | Spd: {motor.rpm} | Cur: {motor.current} | Temp: {motor.temp}")

driver.set_rx_callback(rx_handler)

# === 主循环 ===
try:
    print(f"开始控制 ID {hex(motor.feedback_id)}，目标速度 {TARGET_SPEED}")
    while True:
        # 1. PID 计算
        output_current = pid.update(TARGET_SPEED, motor.rpm)
        
        # 2. 构造 CAN 报文
        # 逻辑：
        # ID 1-4 (0x201-204) -> 使用控制ID 0x200
        # ID 5-8 (0x205-208) -> 使用控制ID 0x1FF
        
        can_payload = [0] * 8
        m_bytes = motor.get_current_bytes(output_current)
        
        send_id = 0x200
        
        if MOTOR_ID <= 4:
            send_id = 0x200
            idx = (MOTOR_ID - 1) * 2
            can_payload[idx] = m_bytes[0]
            can_payload[idx+1] = m_bytes[1]
        else:
            # 针对 0x205 (ID 5)
            send_id = 0x1FF
            idx = (MOTOR_ID - 5) * 2  # ID 5 填在第 0,1 字节
            can_payload[idx] = m_bytes[0]
            can_payload[idx+1] = m_bytes[1]

        # 3. 发送
        driver.send_can_frame(send_id, can_payload)
        
        # 4. 状态监视
        print(f"Tgt:{TARGET_SPEED} | Real:{motor.rpm:5d} | OutCur:{int(output_current):5d} | Temp:{motor.temp}")
            
        time.sleep(0.005) # 200Hz

except KeyboardInterrupt:
    print("Stop.")
    driver.send_can_frame(0x1FF, [0]*8) # 停止 0x1FF 组
    driver.send_can_frame(0x200, [0]*8) # 停止 0x200 组
    driver.running = False