from math import degrees, isnan, pi

class PN:
    def __init__(self, gain: float = 1.0) -> None:
        self.gain = gain

        self.__previous_los: float = float('nan')
        self.__los: float = float('nan')
        self.__desired_heading_dot: float = float('nan')

    @property
    def desired_heading_dot(self) -> float:
        return self.__desired_heading_dot

    def PN(self, /, new_los: float, dt: float) -> None:
        self.__previous_los = self.__los
        self.__los = new_los

        if isnan(self.__previous_los):
            self.__desired_heading_dot = 0.0
            return

        los_dot: float = (self.__los - self.__previous_los) / dt
        self.__desired_heading_dot: float = self.gain * los_dot
        
