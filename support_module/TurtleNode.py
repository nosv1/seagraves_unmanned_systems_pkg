#!usr/env/bin python3

# python imports
from math import degrees

# ros2 imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

# personal imports
from support_module.Logger import Logger

class DetectedObject:
    def __init__(self, distance: float, angle: float):
        self.distance = distance
        self.angle = angle  # (-180, 180) degrees

    def __str__(self):
        return f"Range: {self.distance}, Angle: {self.angle}"

class Turtle(Node):
    def __init__(self, namespace='', name='Turtle') -> None:
        super().__init__(name)

        self.cmd_vel_publisher: Publisher = self.create_publisher(
            Twist, f"{namespace}/cmd_vel", 10)
        self.odom_subscriber: Subscription = self.create_subscription(
            Odometry, f"{namespace}/odom", self.odom_callback, 10)
        self.clock_subscriber: Subscription = self.create_subscription(
            Clock, f"{namespace}/clock", self.clock_callback, 10)
        self.lidar_subscriber: Subscription = self.create_subscription(
            LaserScan, f"{namespace}/scan", self.lidar_callback, 10)

        self.last_callback = None

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

        self.command_logger = Logger(
            headers=["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"], 
            filename=f"{name}_command_log.csv")
        self.heading_logger = Logger(
            headers=["time", "desired_heading", "actual_heading"], 
            filename=f"{name}_heading_log.csv")
        self.pose_logger = Logger(
            headers=["time", "position_x", "position_y", "position_z", "roll", "pitch", "yaw"], 
            filename=f"{name}_pose_log.csv")

    def move(self):
        self.cmd_vel_publisher.publish(self.twist)

        self.command_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.twist.linear.x, 
            self.twist.linear.y, 
            self.twist.linear.z, 
            self.twist.angular.x, 
            self.twist.angular.y, 
            self.twist.angular.z
        ])

    def odom_callback(self, msg: Odometry) -> None:
        self.last_callback = self.odom_callback

        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.previous_wall_time = self.current_wall_time
        self.current_wall_time = self.get_clock().now().nanoseconds / 1e9
        self.dt = self.current_wall_time - self.previous_wall_time

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.position.x, 
            self.position.y, 
            self.position.z, 
            degrees(self.roll),
            degrees(self.pitch),
            degrees(self.yaw)
        ])

    def clock_callback(self, msg: Clock) -> None:
        self.last_callback = self.clock_callback

        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.sim_start_time = self.sim_start_time if self.sim_start_time else self.current_sim_time
        self.sim_elapsed_time = self.current_sim_time - self.sim_start_time

    def lidar_callback(self, msg: LaserScan) -> None:
        self.last_callback = self.lidar_callback

        self.detected_objects: list[DetectedObject] = []
        for i, distance in enumerate(msg.ranges):
            if distance != float("inf"):
                angle = i if i < 180 else i - 360
                detected_object: DetectedObject = DetectedObject(
                    distance=distance, angle=angle)
                self.detected_objects.append(detected_object)