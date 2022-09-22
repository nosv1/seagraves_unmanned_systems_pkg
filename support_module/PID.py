class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def update(self, desired: float, actual: float, dt: float) -> float:
        error = desired - actual
        self.integral += error * dt
        self.derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (
            self.kp * error + 
            self.ki * self.integral + 
            self.kd * self.derivative
        )