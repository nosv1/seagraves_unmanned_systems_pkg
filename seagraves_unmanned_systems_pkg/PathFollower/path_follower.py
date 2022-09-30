#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import degrees, pi

# ros2 imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock

# path follower imports
from Point import Point as PathPoint
from Waypoint import Waypoint
from SearchAlgorithms.Scenario import Scenario

# support module imports
from support_module.Logger import Logger
from support_module.quaternion_tools import euler_from_quaternion
from support_module.PID import PID
from support_module.math_tools import clamp

class PathFollower(Node):
    def __init__(self, PID: PID, waypoints: list[Waypoint]) -> None:
        super().__init__("path_follower")

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

        self.command_logger = Logger(headers=["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"], filename="command_log.csv")
        self.heading_logger = Logger(headers=["time", "desired_heading", "actual_heading"], filename="heading_log.csv")
        self.pose_logger = Logger(headers=["time", "position_x", "position_y", "position_z", "roll", "pitch", "yaw"], filename="pose_log.csv")
        self.waypoint_logger = Logger(headers=["time", "waypoint_x", "waypoint_y", "waypoint_z"], filename="waypoint_log.csv")

        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.PID = PID

        # waypoints are reverse order due to pathfinder returning reverse order path
        self.waypoints: list[Waypoint] = waypoints
        self.current_waypoint_index: int = -1
        self.current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        self.path_complete: bool = False

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
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.previous_wall_time = self.current_wall_time
        self.current_wall_time = self.get_clock().now().nanoseconds / 1e9
        self.dt = self.current_wall_time - self.previous_wall_time

        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w,
        )

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.position.x, 
            self.position.y, 
            self.position.z, 
            degrees(self.roll),
            degrees(self.pitch),
            degrees(self.yaw)
        ])

        self.update()

    def clock_callback(self, msg: Clock) -> None:
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.sim_start_time = self.sim_start_time if self.sim_start_time else self.current_sim_time
        self.sim_elapsed_time = self.current_sim_time - self.sim_start_time

    def switch_waypoint(self):
        """
        try to switch to next waypoint if within waypoint's radius
        """
        current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        if current_waypoint.point_within_radius(self.position):
            print(f"Arrived at waypoint: {current_waypoint}")
            self.waypoint_logger.log([
                self.get_clock().now().nanoseconds / 1e9, 
                current_waypoint.x, 
                current_waypoint.y, 
                current_waypoint.z
            ])
            self.PID.prev_error = 0.0
            # self.PID.integral = 0.0  
            # is this needed? /\/\/\/\
            # or when switching waypoints should you use an average integral 
            # based on past waypoints?
            self.path_complete = current_waypoint == self.waypoints[0]
            if not self.path_complete:
                self.current_waypoint_index += -1
                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                print(f"Next waypoint: {self.current_waypoint}")

    def update(self):
        self.switch_waypoint()

        # get the desired heading
        desired_heading: float = PathPoint(
            x=self.position.x,
            y=self.position.y,
            z=0
        ).heading_to(self.current_waypoint)

        # decide to turn left or right
        desired_heading = (
            desired_heading - 2 * pi
            if desired_heading - self.yaw > pi
            else desired_heading
        )

        self.twist.angular.z = clamp(
            self.PID.update(
                desired=desired_heading, actual=self.yaw, dt=self.dt
            ), -2.84, 2.84
        )
        self.twist.linear.x = 0.95
        self.move()

        self.heading_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            degrees(desired_heading),
            degrees(self.yaw)
        ])
def main() -> None:
    rclpy.init()

    print("Loading scenario...")
    scenario: Scenario = Scenario().loader(
        "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/SearchAlgorithms/scenarios/AStar_15x15_bot-0o5_grid-1o0.json"
        # "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/SearchAlgorithms/scenarios/RRT_15x15_bot-0o5_grid-1o0.json"
    )

    print("Finding path...")
    scenario.algorithm.find_path()

    print("Setting waypoints...")
    # Waypoints are stored in reverse order because the path finder creates the 
    # path starting at goal.
    waypoints: list[Waypoint] = []
    for node in scenario.algorithm.path:
        waypoints.append(Waypoint(x=node.x, y=node.y, radius=0.1))

    print("Initializing path_follower node...")
    path_follower: PathFollower = PathFollower(
        PID=PID(kp=4.5, ki=0.0, kd=0.25),
        waypoints=waypoints
    )

    print("Following waypoints...")
    print(f"Next waypoint: {path_follower.current_waypoint}")
    while rclpy.ok():
        rclpy.spin_once(path_follower)

        if path_follower.path_complete:
            print("Path complete!")
            print(f"Elapsed sim time: {path_follower.sim_elapsed_time}")
            print(f"Speed: {scenario.algorithm.path[0].total_cost / path_follower.sim_elapsed_time}")
            print("Stopping turtlebot...")
            path_follower.twist.angular.z = 0.0
            path_follower.twist.linear.x = 0.0
            path_follower.move()
            break

    path_follower.command_logger.close()
    path_follower.heading_logger.close()
    path_follower.pose_logger.close()

    path_follower.destroy_node()
    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()