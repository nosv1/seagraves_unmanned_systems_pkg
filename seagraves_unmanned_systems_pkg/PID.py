from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class PID(Node):
    def __init__(self) -> None:
        super().__init__("PID")
        self.subscriber = self.create_subscription(Odometry, "/odom", self.callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.msg = Twist()

    def callback(self, msg: Odometry) -> None:
        self.get_logger().info(f"Position: [{msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f}]")
        self.get_logger().info(f"Rotation: [{msg.pose.pose.orientation.x:.2f}, {msg.pose.pose.orientation.y:.2f}, {msg.pose.pose.orientation.z:.2f}, {msg.pose.pose.orientation.w:.2f}]")

        position: Point = msg.pose.pose.position
        orientation: Quaternion = msg.pose.pose.orientation

        if int(position.x) % 2 == 0:
            self.msg.linear.x = .25

        else:
            self.msg.linear.x = .75

        self.publish(self.msg)

    def publish(self, msg: Twist) -> None:
        self.get_logger().info(f"Publishing: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}]")
        self.publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    pid = PID()
    while rclpy.ok():
        rclpy.spin_once(pid)
    pid.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()