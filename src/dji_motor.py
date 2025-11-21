import struct

class DJIMotor:
    def __init__(self, motor_id, can_id_base=0x200):
        self.id = motor_id
        self.feedback_id = can_id_base + motor_id
        
        self.angle = 0
        self.rpm = 0
        self.current = 0
        self.temp = 0

    def parse_feedback(self, raw_data):
        if len(raw_data) < 8: return
        # 大端解析: Angle, RPM, Current, Temp, Null
        (self.angle, self.rpm, self.current, self.temp) = struct.unpack('>hhhB', raw_data[:7])

    def get_current_bytes(self, current_val):
        # 这里的限幅取决于电机型号，3508/6020通常最大16384或30000
        limit = 25000 
        if current_val > limit: current_val = limit
        if current_val < -limit: current_val = -limit
        val = int(current_val)
        return [(val >> 8) & 0xFF, val & 0xFF]