from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(Odometry, "/odom", self.callback, 10)

    def callback(self, msg: Odometry):
        self.get_logger().info(f"Position: {msg.pose.pose.position}")
        self.get_logger().info(f"Rotation: {msg.pose.pose.orientation}")
    

def main():
    rclpy.init()

    publisher_node = OdomSubscriber()

    publisher_node.get_logger().info("Subscriber node started")
    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()