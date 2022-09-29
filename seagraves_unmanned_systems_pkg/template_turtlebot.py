#!usr/env/bin python3

# python imports

# ros2 imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock

# personal imports

class TemplateTurtlebot(Node):
    def __init__(self) -> None:
        super().__init__("template_turtlebot")

        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"/cmd_vel", 10
        )
        self.odom_subscriber: Subscription = self.create_subscription(
            Odometry, f"/odom", self.odom_callback, 10
        )
        self.clock_subscriber: Subscription = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )

        self.twist: Twist = Twist()
        self.position: Point = Point()
        self.orientation: Quaternion = Quaternion()
        self.previous_wall_time: float = 0.0
        self.current_wall_time: float = self.get_clock().now().nanoseconds / 1e9
        self.dt: float = 0.0
        self.sim_current_time: float = 0.0
        self.sim_start_time: float = None
        self.sim_elapsed_time: float = 0.0

    def move(self):
        self.cmd_vel_publisher.publish(self.twist)

    def odom_callback(self, msg: Odometry) -> None:
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.previous_wall_time = self.current_wall_time
        self.current_wall_time = self.get_clock().now().nanoseconds / 1e9
        self.dt = self.current_wall_time - self.previous_wall_time

    def clock_callback(self, msg: Clock) -> None:
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.sim_start_time = self.sim_start_time if self.sim_start_time else self.current_sim_time
        self.sim_elapsed_time = self.current_sim_time - self.sim_start_time

    def update(self):
        return

def main() -> None:
    rclpy.init()

    template_turtlebot: TemplateTurtlebot = TemplateTurtlebot()

    while rclpy.ok():
        rclpy.spin_once(template_turtlebot)
        template_turtlebot.update()

    template_turtlebot.destroy_node()
    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()