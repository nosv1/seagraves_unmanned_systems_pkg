from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.twist = Twist()

    def publish_speed(self, speed: float):
        self.twist.linear.x = speed
        self.publisher_.publish(self.twist)
        self.get_logger().info(f"Publishing speed: {speed}")

    def publish_rotation(self, rotation: float):
        self.twist.angular.z = rotation
        self.publisher_.publish(self.twist)
        self.get_logger().info(f"Publishing rotation: {rotation}")
        
    def move(self, speed: float, duration: float, stop: bool = False):
        duration_ns: float = duration * 1e9
        now_ns: float = self.get_clock().now().nanoseconds
        while self.get_clock().now().nanoseconds - now_ns < duration_ns:
            self.publish_speed(speed)
            rclpy.spin_once(self, timeout_sec=0.1)
        if stop:
            self.publish_speed(0.0)
            rclpy.spin_once(self, timeout_sec=0.1)

    def turn(self, rotation: float, duration: float, stop: bool = False):
        duration_ns: float = duration * 1e9
        now_ns: float = self.get_clock().now().nanoseconds
        while self.get_clock().now().nanoseconds - now_ns < duration_ns:
            self.publish_rotation(rotation)
            rclpy.spin_once(self, timeout_sec=0.1)
        if stop:
            self.publish_rotation(0.0)
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()

    publisher_node = CmdVelPublisher()
    
    publisher_node.move(1.5, 5.0)
    publisher_node.turn(-.15, 2, stop=True)
    publisher_node.move(1.5, 5.0, stop=True)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()