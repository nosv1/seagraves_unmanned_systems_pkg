#!/usr/bin/env python3

from __future__ import annotations

# python imports
from math import degrees, pi
import numpy as np
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
        print(f"----------------------------------------")
        
        los = (np.array([self.evader_position.x, self.evader_position.y]) 
            - np.array([self.position.x, self.position.y]))
        los_as_radians = np.arctan2(los[1], los[0])

        pursuer_vector = (np.array([self.position.x, self.position.y]) 
            - np.array([self.previous_position.x, self.previous_position.y]))
        pursuer_velocity = np.linalg.norm(pursuer_vector) / self.odom_dt
        pursuer_position = np.array([self.position.x, self.position.y])
        pursuer_heading = np.arctan2(pursuer_vector[1], pursuer_vector[0])

        evader_vector = (np.array([self.evader_position.x, self.evader_position.y])
            - np.array([self.evader_previous_position.x, self.evader_previous_position.y]))
        evader_velocity = np.linalg.norm(evader_vector) / self.odom_dt
        evader_position = np.array([self.evader_position.x, self.evader_position.y])
        evader_heading = np.arctan2(evader_vector[1], evader_vector[0])

        previous_los = (np.array([self.evader_previous_position.x, self.evader_previous_position.y])
            - np.array([self.previous_position.x, self.previous_position.y]))
        previous_los_as_radians = np.arctan2(previous_los[1], previous_los[0])

        los = (np.array([self.evader_position.x, self.evader_position.y])
            - np.array([self.position.x, self.position.y]))
        los_as_radians = np.arctan2(los[1], los[0])

        los_delta = los_as_radians - previous_los_as_radians

        rotation_delta = self.PN.gain * los_delta
        desired_heading = pursuer_heading + rotation_delta
        rotation_delta /= self.odom_dt

        # self.PN.PN(
        #     new_los=los_as_radians,
        #     dt=self.odom_dt
        # )

        self.PN.PN(
            evader_pos=self.evader_position,
            puruser_pos=self.position,
            dt=self.odom_dt,
            current_heading=self.yaw
        )
        rotation_delta = self.PN.desired_heading_dot
        
        self.twist.angular.z = clamp(
            rotation_delta,
            -self.max_turn_rate,
            self.max_turn_rate
        )
        # print(f"{self.twist.angular.z=}")

        # decide to turn left or right
        # desired_heading = (
        #     desired_heading - 2 * pi
        #     if desired_heading - self.yaw > pi
        #     else desired_heading
        # )

        # self.twist.angular.z = clamp(
        #     self.heading_PID.update(
        #         desired=desired_heading, actual=self.yaw, dt=self.odom_dt
        #     ), -self.max_turn_rate, self.max_turn_rate
        # )

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

        return None

    def on_lidar_callback(self) -> None:
        if not self.detected_objects:
            return None

        closest_object = min(
            self.detected_objects, 
            key=lambda d_o: max(
                s_p.distance for s_p in d_o.significant_points))
        closest_point = min(
                closest_object.significant_points, 
                key=lambda s_p: s_p.distance).point 

        self.PN.PN(
            evader_pos=closest_point,
            puruser_pos=self.position,
            dt=self.lidar_dt,
            current_heading=self.yaw
        )
        
        self.twist.angular.z = clamp(
            self.PN.desired_heading_dot,
            -self.max_turn_rate,
            self.max_turn_rate
        )
        print(f"{self.evader_position=}")
        print(f"{closest_point.x=}, {closest_point.y=}")
        self.twist.linear.x = self.max_speed
        self.move()

        self.heading_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            degrees(self.twist.angular.z),
            0,
        ])

        return None

    def update(self) -> None:
        if self.last_callback == self.__odom_callback:
            return
            self.on_odom_callback()

        elif self.last_callback == self.__lidar_callback:
            self.on_lidar_callback()

        return None


def main() -> None:
    rclpy.init()

    print("Initializing pursuer node...")
    pursuer: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        PN_gain=5,
        namespace='',
        name="Pursuer")

    pursuer.throttle_PID = None
    pursuer.max_speed = 0.2
    # pursuer.max_speed = 0.0
    pursuer.max_turn_rate = 1.0

    # for subscription in pursuer.subscriptions:
    #     if subscription.topic_name == f"{pursuer.name}/odom":
    #         pursuer.subscriptions.remove(subscription)
        
    #     elif subscription.topic_name == f"{pursuer.name}/clock":
    #         pursuer.subscriptions.remove(subscription)

    print(f"Spinning {pursuer.name} ({pursuer.namespace=})...")
    while rclpy.ok():
        rclpy.spin_once(pursuer)
        pursuer.update()

        if degrees(pursuer.roll) > 1:
            break

    pursuer.close_logs()
    pursuer.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()