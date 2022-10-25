from math import degrees, isnan, pi

class PN:
    def __init__(self, gain: float = 1.0) -> None:
        self.gain = gain

        self.__previous_los: float = float('nan')
        self.__los: float = float('nan')
        self.__desired_heading_dot: float = float('nan')
        self.__desired_heading = float('nan')

    @property
    def desired_heading_dot(self) -> float:
        return self.__desired_heading_dot

    @property
    def desired_heading(self) -> float:
        return self.__desired_heading

    def PN(self, /, new_los: float, dt: float, current_yaw: float) -> None:
        self.__previous_los = self.__los
        self.__los = new_los

        if isnan(self.__previous_los):
            self.__desired_heading_dot = 0.0
            self.__desired_heading = current_yaw
            return

        # how is this PN? isn't this just chasing not intercepting?
        # how is this different to just setting the desired heading for a PID to be the LOS?
        # self.__desired_heading_dot = (self.gain * new_los) / dt

        los_dot: float = (self.__los - self.__previous_los) / dt
        self.__desired_heading_dot: float = self.gain * abs(los_dot)
        self.__desired_heading = (current_yaw 
            + (self.__desired_heading_dot 
                * (-1 if los_dot < 0 else 1)
                * dt))