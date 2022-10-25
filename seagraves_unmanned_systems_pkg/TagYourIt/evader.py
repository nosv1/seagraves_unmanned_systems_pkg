#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import degrees, isnan, pi, radians
from numpy import mean

# ros2 imports
import rclpy

# personal imports
from support_module.DetectedObject import DetectedObject
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
        waypoints: list[Waypoint],
        **kwargs) -> None:
        super().__init__(**kwargs)

        self.heading_PID = heading_PID
        self.throttle_PID = throttle_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate
        self.waypoints = waypoints

        self.closest_object: DetectedObject = None

        self.current_waypoint: Waypoint = self.waypoints[0]

    def on_odom_callback(self) -> None:
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w
        )

        # is evasive
        if self.closest_object.distance < 1:
            # desired heading is what ever heading puts the closest object 90 degress off nose
            actual: float = (
                self.closest_object.angle
                if self.closest_object.angle > 0
                else self.closest_object.angle + 2 * pi
            ) - radians(90)

            self.twist.angular.z = clamp(
                self.heading_PID.update(
                    desired=0, 
                    actual=-actual,
                    dt=self.odom_dt), 
                -self.max_turn_rate, self.max_turn_rate
            )

        # not evasive            
        else:
            # get the desired heading
            desired_heading: float = Point(
                x=self.position.x,
                y=self.position.y,
                z=0
            ).heading_to(self.current_waypoint)

            # decide to turn left or right
            desired_heading = (
                desired_heading - 2 * pi
                if desired_heading - self.yaw > pi
                else desired_heading
            )
            self.twist.angular.z = clamp(
                self.heading_PID.update(
                    desired=desired_heading, actual=self.yaw, dt=self.odom_dt
                ), -self.max_turn_rate, self.max_turn_rate
            )
        self.twist.linear.x = self.max_speed
        self.move()

    def on_lidar_callback(self) -> None:
        self.closest_object = min(
            self.detected_objects, key=lambda d_o: d_o.distance)

    def update(self) -> None:
        if not self.detected_objects:
            return

        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()

        elif self.last_callback == self.__lidar_callback:
            self.on_lidar_callback()


def main() -> None:
    rclpy.init()

    waypoints: list[Waypoint] = [
        Waypoint(x=9, y=9, radius=0.1)
    ]

    evader: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        waypoints=waypoints,
        namespace='turtle',
        name="Evader")

    evader.throttle_PID = None
    evader.max_speed = 0.1
    # evader.max_turn_rate = 1.0

    # for subscription in pursuer.subscriptions:
    #     if subscription.topic_name == f"{pursuer.name}/odom":
    #         pursuer.subscriptions.remove(subscription)
        
    #     elif subscription.topic_name == f"{pursuer.name}/clock":
    #         pursuer.subscriptions.remove(subscription)

    while rclpy.ok():
        rclpy.spin_once(evader)
        evader.update()

        if degrees(evader.roll) > 1:
            break

    evader.close_logs()
    evader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()