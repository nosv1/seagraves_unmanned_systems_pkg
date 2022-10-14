#!usr/env/bin python3

# python imports

# ros2 imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

# personal imports

class DetectedObject:
    def __init__(self, distance: float, angle: float):
        self.distance = distance
        self.angle = angle

    def __str__(self):
        return f"Range: {self.distance}, Angle: {self.angle}"

class Turtle(Node):
    def __init__(self, namespace='turtle', name='') -> None:
        super().__init__(namespace)

        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"{name}/cmd_vel", 10)
        self.odom_subscriber: Subscription = self.create_subscription(
            Odometry, f"{name}/odom", self.odom_callback, 10)
        self.clock_subscriber: Subscription = self.create_subscription(
            Clock, f"{name}/clock", self.clock_callback, 10)
        self.lidar_subscriber: Subscription = self.create_subscription(
            LaserScan, f"{name}/scan", self.lidar_callback, 10)

        self.twist: Twist = Twist()

        self.position: Point = Point()
        self.orientation: Quaternion = Quaternion()
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0

        self.previous_wall_time: float = 0.0
        self.current_wall_time: float = self.get_clock().now().nanoseconds / 1e9
        self.dt: float = 0.0
        self.sim_current_time: float = 0.0
        self.sim_start_time: float = None
        self.sim_elapsed_time: float = 0.0
        self.detected_objects: list[DetectedObject] = []

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

    def lidar_callback(self, msg: LaserScan) -> None:
        self.detected_objects: list[DetectedObject] = []
        for i, distance in enumerate(msg.ranges):
            if distance != float("inf"):
                detected_object: DetectedObject = DetectedObject(
                    distance=distance, angle=i)
                self.detected_objects.append(detected_object)