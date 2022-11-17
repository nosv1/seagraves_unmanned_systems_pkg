from math import degrees, isnan, pi
from numpy import arctan2, array, ndarray

from .Point import Point

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

    def PN(self, /, evader_pos: Point, puruser_pos: Point, dt: float, current_heading: float) -> None:
        los_vector = Point(*(
            array([evader_pos.x, evader_pos.y, evader_pos.z])
            - array([puruser_pos.x, puruser_pos.y, puruser_pos.z])),
        )
        self.__previous_los = self.__los
        self.__los = arctan2(los_vector.y, los_vector.x)
        # TODO consider when we lose track of target for an iteration...

        if isnan(self.__los): # first time through
            self.__desired_heading_dot = 0.0
            self.__desired_heading = current_heading
            return

        los_delta: float = self.__los - self.__previous_los
        self.__desired_heading_dot = (self.gain * los_delta) / dt
        self.__desired_heading = current_heading + (self.__desired_heading_dot * dt)

        return None