#!usr/bin/env python

from __future__ import annotations

# python imports
from math import pi

# ros imports
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

# TODO: turtlebot 1 turn from 0 to 5 seconds
# TODO: turltebot 2 turn from 5 to 10 seconds

class DetectedObject:
    def __init__(self, distance: float, angle: float):
        self.distance = distance
        self.angle = angle

    def __str__(self):
        return f"Range: {self.distance}, Angle: {self.angle}"

class Turtlebot(Node):
    def __init__(self, name) -> None:
        super().__init__(name)
        
        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"{name}/cmd_vel", 10)
        self.clock_susbscriber: Subscription = self.create_subscription(
            Clock, f"{name}/clock", self.clock_callback, 10)
        self.lidar_subscriber: Subscription = self.create_subscription(
            LaserScan, f"{name}/scan", self.lidar_callback, 10)

        self.detected_objects: list[DetectedObject] = []

        self.msg: Twist = Twist()

    def clock_callback(self, msg: Clock) -> None:
        return

    def lidar_callback(self, msg: LaserScan) -> None:
        self.detected_objects: list[DetectedObject] = []
        for i, distance in enumerate(msg.ranges):
            if distance != float("inf"):
                detected_object: DetectedObject = DetectedObject(
                    distance=distance, angle=i)
                self.detected_objects.append(detected_object)
                print(detected_object)
        print()

    def move(
        self, linear_velocity: float=None, angular_velocity: float=None
    ) -> None:
        if linear_velocity:
            self.msg.linear.x = linear_velocity
        if angular_velocity:        
            self.msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(self.msg)

def main() -> None:
    rclpy.init()

    # names of turtles need to match the spawned bot's name
    # maybe 'namespace' is a thing?
    # turtlebot1 = Turtlebot("")
    turtlebot2 = Turtlebot("turtle")

    # start: float = turtlebot1.get_clock().now().nanoseconds / 1e9

    while rclpy.ok():
        # now: float = turtlebot1.get_clock().now().nanoseconds / 1e9
        # ellapsed: float = now - start
        rclpy.spin_once(turtlebot2)
        turtlebot2.move(angular_velocity=pi/2)

    # turtlebot1.move(linear_velocity=0, angular_velocity=0)
    # turtlebot2.move(linear_velocity=0, angular_velocity=0)

    # turtlebot1.destroy_node()
    turtlebot2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()