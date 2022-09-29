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

# TODO: turtlebot 1 turn from 0 to 5 seconds
# TODO: turltebot 2 turn from 5 to 10 seconds

class Turtlebot(Node):
    def __init__(self, name) -> None:
        super().__init__(name)
        
        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"{name}/cmd_vel", 10
        )
        self.clock_susbscriber: Subscription = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )

        self.msg: Twist = Twist()

    def clock_callback(self, msg: Clock) -> None:
        return

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
    turtlebot1 = Turtlebot("")
    turtlebot2 = Turtlebot("turtle")

    start: float = turtlebot1.get_clock().now().nanoseconds / 1e9

    while rclpy.ok():
        now: float = turtlebot1.get_clock().now().nanoseconds / 1e9
        ellapsed: float = now - start
        
        if 0 <= ellapsed <= 5:
            turtlebot1.move(angular_velocity=0.5)
        elif 5 <= ellapsed <= 10:
            turtlebot2.move(angular_velocity=1.0)
        else:
            break

    # turtlebot1.move(linear_velocity=0, angular_velocity=0)
    # turtlebot2.move(linear_velocity=0, angular_velocity=0)

    turtlebot1.destroy_node()
    turtlebot2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()