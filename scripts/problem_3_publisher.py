import time

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

    def publish_rotation(self, rotation: float):
        self.twist.angular.z = rotation
        self.publisher_.publish(self.twist)
        
    def move(self, speed: float, duration: float, stop: bool = False):
        self.get_logger().info(f"Moving @ {speed}m/s for {duration}s...")
        
        duration_ns: float = duration * 1e9
        sim_now_ns: int = self.get_clock().now().nanoseconds
        pc_now_ns: float = time.time_ns()

        while self.get_clock().now().nanoseconds - sim_now_ns < duration_ns:
            self.publish_speed(speed)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info(f"Sim Ellapsed: {self.get_clock().now().nanoseconds - sim_now_ns}")
        self.get_logger().info(f"PC Ellapsed: {time.time_ns() - pc_now_ns}")

        if stop:
            self.get_logger().info("Moving stopped...")
            self.publish_speed(0.0)
            rclpy.spin_once(self, timeout_sec=0.1)

    def turn(self, rotation: float, duration: float, stop: bool = False):
        self.get_logger().info(f"Turning @ {rotation}rad/s for {duration}s...")
        
        duration_ns: float = duration * 1e9
        sim_now_ns: int = self.get_clock().now().nanoseconds
        pc_now_ns: float = time.time_ns()

        while self.get_clock().now().nanoseconds - sim_now_ns < duration_ns:
            self.publish_rotation(rotation)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info(f"Sim Ellapsed: {self.get_clock().now().nanoseconds - sim_now_ns}")
        self.get_logger().info(f"PC Ellapsed: {time.time_ns() - pc_now_ns}")
            
        if stop:
            self.get_logger().info("Turning stopped...")
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