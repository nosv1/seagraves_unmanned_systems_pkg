
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_speed(self, speed: float):
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing speed: {speed}")

    def publish_rotation(self, rotation: float):
        msg = Twist()
        msg.angular.z = rotation
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing rotation: {rotation}")
        
    def move(self, speed: float, duration: float):
        now: float = time.perf_counter()
        self.publish_speed(speed)
        rclpy.spin_once(self, timeout_sec=0.1)
        while time.perf_counter() < now + duration:
            continue
        # self.publish_speed(0.0)
        # rclpy.spin_once(self, timeout_sec=0.1)

    def turn(self, rotation: float, duration: float):
        now: float = time.perf_counter()
        self.publish_rotation(rotation)
        rclpy.spin_once(self, timeout_sec=0.1)
        while time.perf_counter() < now + duration:
            continue
        self.publish_rotation(0.0)
        rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()

    publisher_node = CmdVelPublisher()
    
    while rclpy.ok():
        publisher_node.move(1.5, 5.0)
        publisher_node.turn(-.15, 2)
        publisher_node.move(1.5, 5.0)
        publisher_node.publish_speed(0.0)
        break

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()