import time

class PID:
    def __init__(self, kp, ki, kd, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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
        
        # P
        p_term = self.kp * error
        
        # I (抗饱和)
        self.integral += error * dt
        # 简单的 I 限幅
        i_limit = self.out_max / (self.ki if self.ki > 0 else 1.0) 
        if self.integral > i_limit: self.integral = i_limit
        if self.integral < -i_limit: self.integral = -i_limit
        
        i_term = self.ki * self.integral
        
        # D
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        output = p_term + i_term + d_term
        
        # 总输出限幅
        if output > self.out_max: output = self.out_max
        if output < -self.out_max: output = -self.out_max
            
        return output