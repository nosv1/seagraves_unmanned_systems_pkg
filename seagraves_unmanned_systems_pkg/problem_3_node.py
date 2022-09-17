import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Problem3Node(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_speed(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing speed: {speed}")

def main():
    rclpy.init()

    publisher_node = Problem3Node()

    speed = 1.5
    while rclpy.ok():
        publisher_node.publish_speed(speed)
        rclpy.spin_once(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()