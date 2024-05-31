
class LongitudinalController():

    """
    
    Longitudinal PID controller
    
    
    """

    def __init__(self, dt, kp = 1, ki = 0.2, kd = 0.01) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error = 0
        self.previous_integral_error = 0
        self.v = 0
        self.dt = dt

    def update_speed(self, v_desired):
        # error term
        self.error = v_desired - self.v

        # I
        if (self.error < 0 and self.previous_error >0) or (self.error > 0 and self.previous_error <0):
            self.integral_error = 0
        else:
            self.integral_error = self.previous_integral_error + self.error * self.dt

        # D
        derivate = (self.error - self.previous_error) / self.dt

        acc = self.kp * self.error + self.ki * self.integral_error + self.kd * derivate
        self.v = self.v + acc * self.dt

        self.previous_error = self.error
        self.previous_integral_error = self.integral_error

        return self.v