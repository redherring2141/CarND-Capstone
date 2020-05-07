
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, max_input=MAX_NUM, mn=MIN_NUM, mx=MAX_NUM):
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
        self.t = 0.0


    #def step(self, error, sample_time):
    def step(self, value_target, value_curr, sample_time):
        
        if self.t == None:
            self.t = sample_time
            self.integral = 0.0
            self.error = value_target - value_curr
            return 0.0

        delta_t = sample_time - self.t

        # Calculate error, integral, derivative
        error = value_target - value_curr
        integral = max(MIN_NUM, min(MAX_NUM, self.integral + error*delta_t))
        derivative = (error - self.error) / delta_t

        # Calculate PID control
        control = max(self.min, min(self.max, (self.kp * error + self.ki * integral + self.kd * derivative)))

        self.t = delta_t
        self.error = error
        self.integral = integral        

        return control


        '''
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
        '''
