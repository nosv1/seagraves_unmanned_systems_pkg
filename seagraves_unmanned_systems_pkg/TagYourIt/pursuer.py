#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import degrees, isnan, pi, radians
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
        PN_gain: float,
        **kwargs) -> None:
        super().__init__(**kwargs)

        self.heading_PID = heading_PID
        self.throttle_PID = throttle_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate
        self.PN_gain = PN_gain
        
        self.PN = PN(gain=PN_gain)

    def on_odom_callback(self) -> None:
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w
        )

        print(f"Desired heading dot: {degrees(self.PN.desired_heading_dot)}")

        self.twist.angular.z = clamp(
            self.PN.desired_heading_dot,
            -self.max_turn_rate,
            self.max_turn_rate
        )

        if self.throttle_PID:
            distance_to: float = mean([
                d_o.distance for d_o in self.detected_objects])
            self.twist.linear.x = clamp(
                self.max_speed - self.throttle_PID.update(
                    desired=1.5, actual=distance_to, dt=self.odom_dt
                ), 0.0, self.max_speed
            )

        else:
            self.twist.linear.x = self.max_speed

        self.move()

        # self.heading_logger.log([
        #     self.get_clock().now().nanoseconds / 1e9, 
        #     degrees(self.yaw),
        #     degrees(self.PN.desired_heading),
        # ])

    def on_lidar_callback(self) -> None:
        # get angle of closest object
        relative_angle: float = radians(min(
            self.detected_objects, key=lambda d_o: d_o.distance).angle)

        # convert [0, 360) to (-180, 180]
        relative_angle = (
            relative_angle - 2 * pi
            if relative_angle > pi
            else relative_angle
        )

        self.PN.PN(
            new_los=relative_angle,
            dt=self.lidar_dt
        )

        self.twist.angular.z = self.PN.desired_heading_dot
        self.twist.linear.x = self.max_speed
        self.move()

    def update(self) -> None:
        if not self.detected_objects:
            return

        if self.last_callback == self.__odom_callback:
            return
            self.on_odom_callback()

        elif self.last_callback == self.__lidar_callback:
            self.on_lidar_callback()


def main() -> None:
    rclpy.init()

    pursuer: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        PN_gain=0.3,
        namespace='',
        name="Pursuer")

    pursuer.throttle_PID = None
    pursuer.max_speed = 0.2
    pursuer.max_turn_rate = 1.0

    # for subscription in pursuer.subscriptions:
    #     if subscription.topic_name == f"{pursuer.name}/odom":
    #         pursuer.subscriptions.remove(subscription)
        
    #     elif subscription.topic_name == f"{pursuer.name}/clock":
    #         pursuer.subscriptions.remove(subscription)

    while rclpy.ok():
        rclpy.spin_once(pursuer)
        pursuer.update()

    pursuer.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()