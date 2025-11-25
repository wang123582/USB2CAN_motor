# damiao_motor.py
# 达妙电机类，支持 MIT 模式控制与反馈解析

from motor_base import MotorBase
from utils import float_to_uint, uint_to_float, clip

class DamiaoMotor(MotorBase):
    def __init__(self, bus_id, can_id, model="DM4310"):
        """
        :param can_id: 电机配置的 CAN_ID (例如 0x01, 0x02)
        :param model: DM4310, DM4340
        """
        super().__init__(bus_id, can_id)
        self.model = model
        
        # 控制 ID 和 反馈 ID
        # 控制帧 ID = CAN_ID (例如 1)
        # 反馈帧 ID = Master_ID (默认为0) << 8 | CAN_ID ? 
        # 根据您提供的文档: ID 表示控制器ID，取 CAN_ID 低8位。
        # 通常达妙反馈帧的 ID 的低8位就是电机 ID。我们主要通过数据中的 ID 字段或者 CAN 帧 ID 判定。
        # 这里假设 CAN 接收到的 frame_id 的低 8 位即为电机 ID。
        self.control_can_id = can_id 
        
        # MIT 模式控制参数缓存
        self.p_des = 0.0
        self.v_des = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.t_ff = 0.0
        
        # === 达妙参数配置 (必须与上位机一致 !!!) ===
        # 请根据您的上位机设置修改这些值
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.T_MIN = -10.0
        self.T_MAX = 10.0
        
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0
        self.KD_MAX = 5.0
        
        # 状态扩展
        self.err_code = 0
        self.temp_mos = 0
        self.temp_rotor = 0

    def update_feedback(self, can_id, data):
        """
        解析达妙反馈帧 (8 字节)
        ID|ERR, POS_H, POS_L, VEL_H, VEL_L|T_H, T_L, T_MOS, T_ROTOR
        """
        # 校验 ID: 反馈帧的 ID (can_id) 的低8位通常是 motor_id，或者 payload[0] 的高4位是 ID
        # 根据文档: D[0] = (ID & 0x0F) | (ERR << 4) ?? 文档写的是 MST_ID ID | ERR << 4 
        # 让我们按照文档: ID 表示控制器的 ID，取 CAN_ID 的低 8 位。
        # 这里的检查逻辑放宽，主要由上层路由决定是否调用此函数
        
        self.online = True
        
        # Unpack Data
        id_err = data[0]
        self.err_code = (id_err >> 4) & 0x0F
        motor_id_fb = id_err & 0x0F
        
        # P_int (16 bit)
        p_int = (data[1] << 8) | data[2]
        # V_int (12 bit)
        v_int = (data[3] << 4) | (data[4] >> 4)
        # T_int (12 bit)
        t_int = ((data[4] & 0x0F) << 8) | data[5]
        
        self.temp_mos = data[6]
        self.temp_rotor = data[7]
        
        # Convert to float
        self.angle = uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        self.speed = uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        self.torque = uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)
        # (angle 单位是弧度，如果需要角度需转换)
        self.angle_deg = self.angle * 57.29578

    def set_mit_command(self, p_des, v_des, kp, kd, t_ff):
        """设置 MIT 模式控制目标"""
        self.p_des = p_des
        self.v_des = v_des
        self.kp = kp
        self.kd = kd
        self.t_ff = t_ff

    def get_control_frame(self):
        """
        打包 MIT 控制帧
        D0: P[15:8]
        D1: P[7:0]
        D2: V[11:4]
        D3: V[3:0] | Kp[11:8]
        D4: Kp[7:0]
        D5: Kd[11:4]
        D6: Kd[3:0] | T[11:8]
        D7: T[7:0]
        """
        p_int = float_to_uint(self.p_des, self.P_MIN, self.P_MAX, 16)
        v_int = float_to_uint(self.v_des, self.V_MIN, self.V_MAX, 12)
        kp_int = float_to_uint(self.kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = float_to_uint(self.kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = float_to_uint(self.t_ff, self.T_MIN, self.T_MAX, 12)
        
        buf = bytearray(8)
        buf[0] = (p_int >> 8) & 0xFF
        buf[1] = p_int & 0xFF
        buf[2] = (v_int >> 4) & 0xFF
        buf[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
        buf[4] = kp_int & 0xFF
        buf[5] = (kd_int >> 4) & 0xFF
        buf[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
        buf[7] = t_int & 0xFF
        
        return self.control_can_id, buf

    def enable(self):
        """发送使能帧: FC"""
        # D0-D6: FF, D7: FC
        buf = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        return self.control_can_id, buf

    def disable(self):
        """发送失能帧: FD"""
        buf = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
        return self.control_can_id, buf
        
    def save_zero(self):
        """设置零点: FE"""
        buf = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        return self.control_can_id, buf