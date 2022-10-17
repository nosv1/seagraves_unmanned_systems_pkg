#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import degrees, isnan, pi, radians
from statistics import mean

# ros2 imports
import rclpy

# personal imports
from support_module.Logger import Logger
from support_module.math_tools import clamp
from support_module.PID import PID
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

        self.__previous_los: float = float('nan')
        self.__los: float = float('nan')

    def switch_waypoint(self):
        """
        try to switch to next waypoint if within waypoint's radius
        """
        current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        if current_waypoint.point_within_radius(self.position):

            self.heading_PID.prev_error = 0.0
            self.heading_PID.integral = 0.0
            # is this needed? /\/\/\/\
            # or when switching waypoints should you use an average integral 
            # based on past waypoints?

            self.path_complete = current_waypoint == self.waypoints[0]
            if not self.path_complete:
                self.current_waypoint_index += -1
                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                print(
                    f"Next Waypoint {self.current_waypoint_index * -1} / {len(self.waypoints)}: {self.current_waypoint}"
                )

    def PN(self) -> float:
        relative_angle: float = radians(mean([
            d_o.angle for d_o in self.detected_objects]))

        if relative_angle > pi:
            relative_angle -= 2 * pi

        self.__previous_los = self.__los
        self.__los = relative_angle

        if isnan(self.__previous_los):
            return self.yaw

        delta_los: float = self.__los - self.__previous_los
        desired_heading: float = self.yaw + self.PN_gain * delta_los

        desired_heading = (
            desired_heading - 2 * pi
            if desired_heading - self.yaw > pi
            else desired_heading
        )

        return desired_heading

    def update(self) -> None:
        # self.switch_waypoint()

        if self.last_callback != self.lidar_callback:
            return

        if not self.detected_objects:
            return

        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0

        # get the desired heading
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w
        )

        desired_heading: float = self.PN()
        print(f"desired_heading: {degrees(desired_heading)}")
        self.twist.angular.z = clamp(
            self.heading_PID.update(
                desired=desired_heading, actual=self.yaw, dt=self.lidar_dt
            ), -self.max_turn_rate, self.max_turn_rate
        )

        if self.throttle_PID:
            distance_to: float = mean([
                d_o.distance for d_o in self.detected_objects])
            self.twist.linear.x = clamp(
                self.max_speed - self.throttle_PID.update(
                    desired=1.5, actual=distance_to, dt=self.lidar_dt
                ), 0.0, self.max_speed
            ) 

        else:
            self.twist.linear.x = self.max_speed

        self.move()

        self.heading_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            degrees(self.yaw),
            degrees(desired_heading),
        ])

def main() -> None:
    rclpy.init()

    pursuer: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        PN_gain=2,
        namespace='',
        name="Pursuer")

    for subscription in pursuer.subscriptions:
        if subscription.topic_name == f"{pursuer.name}/odom":
            pursuer.subscriptions.remove(subscription)
        
        elif subscription.topic_name == f"{pursuer.name}/clock":
            pursuer.subscriptions.remove(subscription)

    pursuer.throttle_PID = None
    pursuer.max_speed = 0.45

    while rclpy.ok():
        rclpy.spin_once(pursuer)
        pursuer.update()

    pursuer.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()