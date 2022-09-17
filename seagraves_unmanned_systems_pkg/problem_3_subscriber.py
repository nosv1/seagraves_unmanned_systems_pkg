from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(Odometry, "/odom", self.callback, 10)

    def callback(self, msg: Odometry):
        position: Point = msg.pose.pose.position
        rotation: Quaternion = msg.pose.pose.orientation
        self.get_logger().info(f"Position: {position.x:.2f}, {position.y:.2f}, {position.z:.2f}")
        self.get_logger().info(f"Rotation: {rotation.x:.2f}, {rotation.y:.2f}, {rotation.z:.2f}, {rotation.w:.2f}")
    

def main():
    rclpy.init()

    publisher_node = OdomSubscriber()

    publisher_node.get_logger().info("Subscriber node started")
    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()