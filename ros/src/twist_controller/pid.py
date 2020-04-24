
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

        # Approximation - added
        self.max_abs_u = (abs(self.kp) + abs(self.ki) + abs(self.kd)) * abs(max_input)

        # Controller state initialization
        self.t = None
        self.error = 0.0
        self.integral = 0.0


    def reset(self):
        self.int_val = 0.0


    def step(self, error, sample_time):
        
        if self.t == None:
            self.t = t

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        val = self.kp * error + self.ki * integral + self.kd * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
