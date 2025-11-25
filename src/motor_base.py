# motor_base.py
# 电机基类，定义所有电机共有的接口和属性

class MotorBase:
    def __init__(self, bus_id, motor_id):
        self.bus_id = bus_id      # 挂载在哪条 CAN 总线上 (0, 1...)
        self.motor_id = motor_id  # 电机自身的 ID (拨码或软件设定)
        
        # 基础状态
        self.angle = 0.0
        self.speed = 0.0
        self.torque = 0.0
        self.temp = 0.0
        self.online = False       # 是否收到过反馈
    
    def update_feedback(self, can_id, data):
        """解析接收到的 CAN 数据"""
        pass

    def get_control_frame(self):
        """获取发送给 CAN 总线的控制数据 (返回: control_can_id, data_bytes)"""
        pass
    
    def enable(self):
        """使能 (部分电机需要)"""
        pass
        
    def disable(self):
        """失能"""
        pass