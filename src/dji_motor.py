import struct
from motor_base import MotorBase

class DJIMotor(MotorBase):
    def __init__(self, bus_id, motor_id, model):
        """
        DJI 电机驱动类 (支持 GM6020 / GM3508)
        
        :param bus_id: 挂载的 CAN 总线 ID (0, 1...)
        :param motor_id: 电机物理 ID (拨码开关设定值)
        :param model: 电机型号字符串 "GM6020" 或 "GM3508"
        """
        super().__init__(bus_id, motor_id)
        self.model = model
        self.target_output = 0 # 缓存 PID 计算后的控制量 (电压或电流)

        # === ID 映射与参数配置 ===
        
        if model == "GM6020":
            # GM6020 (电压控制)
            # 反馈 ID: ID 1 -> 0x205 ...
            self.feedback_id = 0x205 + (motor_id - 1)
            
            # 控制 ID: 
            # ID 1-4 -> 0x1FF
            # ID 5-8 -> 0x2FF
            self.control_id = 0x1FF if motor_id <= 4 else 0x2FF
            
            # 限幅 (电压)
            self.max_out = 30000

        elif model == "GM3508":
            # GM3508 (电流控制, C620电调)
            # 反馈 ID: ID 1 -> 0x201 ...
            self.feedback_id = 0x201 + (motor_id - 1)
            
            # 控制 ID:
            # ID 1-4 -> 0x200
            # ID 5-8 -> 0x1FF  (注意：这里会与 GM6020 ID 1-4 冲突，需物理避开)
            self.control_id = 0x200 if motor_id <= 4 else 0x1FF
            
            # 限幅 (电流)
            self.max_out = 16384
        
        else:
            raise ValueError(f"[DJIMotor] 未知型号: {model}")

    def update_feedback(self, can_id, data):
        """
        解析 DJI 标准反馈帧 (8字节)
        格式 (Big Endian): Angle(2) RPM(2) Current(2) Temp(1) Null(1)
        """
        # 严格校验 CAN ID
        if can_id != self.feedback_id: 
            return

        self.online = True # 标记电机在线
        
        # 解包
        (raw_angle, raw_rpm, raw_cur, temp) = struct.unpack('>hhhB', data[:7])
        
        # 更新状态属性 (MotorBase 属性)
        # 将 0-8191 映射为 0-360 度
        self.angle = (raw_angle / 8191.0) * 360.0
        self.speed = raw_rpm
        self.torque = raw_cur
        self.temp = temp

    def set_output(self, value):
        """
        设置目标输出值 (由 PID 计算得出)
        :param value: GM6020为电压值, GM3508为电流值
        """
        # 通用限幅逻辑
        if value > self.max_out: value = self.max_out
        if value < -self.max_out: value = -self.max_out
        
        self.target_output = int(value)

    def get_control_bytes(self):
        """
        获取 2 字节控制数据 (Big Endian)
        用于 Main Loop 中的拼包逻辑 (0x1FF / 0x200)
        """
        val = self.target_output
        return [(val >> 8) & 0xFF, val & 0xFF]

    def get_control_frame(self):
        """
        MotorBase 接口实现
        注: DJI 电机通常使用共享帧 (0x1FF/0x200) 统一发送，
        不建议单电机独立发送，因此这里返回 None。
        发送逻辑由 Main Loop 里的拼包器接管。
        """
        return None, None