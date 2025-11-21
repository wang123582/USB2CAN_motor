import time

class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()

    def update(self, target, current):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: dt = 0.001
        self.last_time = now

        error = target - current
        
        # 比例 P
        p_out = self.kp * error
        
        # 积分 I (带抗饱和)
        self.integral += error * dt
        if self.integral > self.out_max / (self.ki + 1e-6):
            self.integral = self.out_max / (self.ki + 1e-6)
        elif self.integral < self.out_min / (self.ki + 1e-6):
            self.integral = self.out_min / (self.ki + 1e-6)
        i_out = self.ki * self.integral
        
        # 微分 D
        d_out = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        output = p_out + i_out + d_out
        
        # 输出限幅
        if output > self.out_max: output = self.out_max
        if output < self.out_min: output = self.out_min
            
        return output