#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import degrees, pi
from numpy import mean

# ros2 imports
import rclpy

# personal imports
from support_module.Logger import Logger
from support_module.math_tools import clamp
from support_module.PID import PID
from support_module.PN import PN
from support_module.Point import Point
from support_module.quaternion_tools import euler_from_quaternion
from support_module.TurtleNode import Turtle as TurtleNode
from support_module.Waypoint import Waypoint

class Turtle(TurtleNode):
    def __init__(self, 
        heading_PID: PID, 
        throttle_PID: PID, 
        max_speed: float, 
        max_turn_rate: float,
        **kwargs) -> None:
        super().__init__(**kwargs)

        self.heading_PID = heading_PID
        self.throttle_PID = throttle_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

    def on_odom_callback(self) -> None:
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w
        )

        return None

    def on_lidar_callback(self) -> None:
        if not self.detected_objects:
            return

        return None

    def update(self) -> None:
        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()

        elif self.last_callback == self.__lidar_callback:
            self.on_lidar_callback()

        return None


def main() -> None:
    rclpy.init()

    tester: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        namespace='',
        name="Tester")

    tester.throttle_PID = None
    tester.max_speed = 0.165
    # pursuer.max_turn_rate = 1.0

    # for subscription in pursuer.subscriptions:
    #     if subscription.topic_name == f"{pursuer.name}/odom":
    #         pursuer.subscriptions.remove(subscription)
         
    #     elif subscription.topic_name == f"{pursuer.name}/clock":
    #         pursuer.subscriptions.remove(subscription)

    while rclpy.ok():
        rclpy.spin_once(tester)
        tester.update()

        if degrees(tester.roll) > 1:
            break

    tester.close_logs()
    tester.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()