#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import pi

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
from seagraves_unmanned_systems.SearchAlgorithms.Scenario import Scenario

# support module imports
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
        self.current_sim_time: float = 0.0

        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.PID = PID

        self.waypoints: list[Waypoint] = waypoints
        self.current_waypoint_index: int = 0
        self.current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        self.path_complete: bool = False

    def move(self):
        self.cmd_vel_publisher.publish(self.twist)

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

        self.update()

    def clock_callback(self, msg: Clock) -> None:
        self.previous_sim_time = self.current_sim_time
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def switch_waypoint(self):
        """
        try to switch to next waypoint if within waypoint's radius
        """
        current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        if current_waypoint.point_within_radius(self.position):
            self.path_complete = current_waypoint == self.waypoints[-1]
            if not self.path_complete:
                self.current_waypoint_index += 1
                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                print(f"switched to waypoint {self.current_waypoint_index}")

    def update(self):
        self.switch_waypoint()

        desired_heading: float = PathPoint(
            x=self.position.x,
            y=self.position.y,
            z=0
        ).heading_to(self.current_waypoint)
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
        self.twist.linear.x = 0.6
        self.move()

def main() -> None:
    rclpy.init()

    print("Loading scenario...")
    scenario: Scenario = Scenario().loader(
        "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/seagraves_unmanned_systems/SearchAlgorithms/scenarios/AStar_15x15_bot-0o5_grid-1o0.json"
        # "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/seagraves_unmanned_systems/SearchAlgorithms/scenarios/RRT_15x15_bot-0o5_grid-1o0.json"
    )

    print("Finding path...")
    scenario.algorithm.find_path()

    print("Setting waypoints...")
    waypoints: list[Waypoint] = []
    for node in scenario.algorithm.path:
        waypoints.insert(0, Waypoint(x=node.x, y=node.y, radius=0.1))

    print("Initializing path_follower node...")
    path_follower: PathFollower = PathFollower(
        PID=PID(kp=4.5, ki=0.0, kd=0.25),
        waypoints=waypoints
    )

    print("Following waypoints...")
    print(f"Waypoints: {' | '.join(str(w) for w in waypoints)}")
    while rclpy.ok():
        # updating before spinning to ensure current waypoint is correct
        path_follower.update()

        # wait for a subscriber callback
        rclpy.spin_once(path_follower)

    path_follower.destroy_node()
    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()