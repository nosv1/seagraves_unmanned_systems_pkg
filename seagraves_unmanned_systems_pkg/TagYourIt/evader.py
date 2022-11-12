#!usr/env/bin python3

from __future__ import annotations

# python imports
from math import degrees, isnan, pi, radians
from numpy import mean

# ros2 imports
import rclpy

# path follower imports
from SearchAlgorithms.Scenario import Scenario

# personal imports
from support_module.DetectedObject import DetectedObject
from support_module.Logger import Logger
from support_module.math_tools import clamp
from support_module.PID import PID
from support_module.PN import PN
from support_module.Point import Point as PathPoint
from support_module.quaternion_tools import euler_from_quaternion
from support_module.TurtleNode import Turtle as TurtleNode
from support_module.Waypoint import Waypoint

class Turtle(TurtleNode):
    def __init__(self, 
        heading_PID: PID, 
        throttle_PID: PID, 
        max_speed: float, 
        max_turn_rate: float,
        waypoints: list[Waypoint],
        **kwargs) -> None:
        super().__init__(**kwargs)

        self.heading_PID = heading_PID
        self.throttle_PID = throttle_PID
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate
        self.waypoints = waypoints

        self.closest_object: DetectedObject = None

        self.current_waypoint: Waypoint = self.waypoints[0]
        self.current_waypoint_index: int = -1
        self.current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        self.path_complete: bool = False

    def switch_waypoint(self):
        """
        try to switch to next waypoint if within waypoint's radius
        """
        current_waypoint: Waypoint = self.waypoints[self.current_waypoint_index]
        if current_waypoint.point_within_radius(self.position):
            # self.waypoint_logger.log([
            #     self.get_clock().now().nanoseconds / 1e9, 
            #     current_waypoint.x, 
            #     current_waypoint.y, 
            #     current_waypoint.z
            # ])

            self.heading_PID.prev_error = 0.0
            self.heading_PID.integral = 0.0

            self.path_complete = current_waypoint == self.waypoints[0]
            if not self.path_complete:
                self.current_waypoint_index += -1
                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                print(
                    f"Next Waypoint {self.current_waypoint_index * -1} / {len(self.waypoints)}: {self.current_waypoint}"
                )

    def on_odom_callback(self) -> None:
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            x=self.orientation.x,
            y=self.orientation.y,
            z=self.orientation.z,
            w=self.orientation.w
        )

        # waypoint following
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
            self.heading_PID.update(
                desired=desired_heading, actual=self.yaw, dt=self.odom_dt
            ), -self.max_turn_rate, self.max_turn_rate
        )

        if self.throttle_PID:
            self.twist.linear.x = clamp(
                self.max_speed - self.throttle_PID.update(
                    desired=desired_heading, actual=self.yaw, dt=self.odom_dt
                ), 0.0, self.max_speed
            )

        else:
            self.twist.linear.x = self.max_speed

        # # is evasive
        # if self.closest_object.distance < 1:
        #     # desired heading is what ever heading puts the closest object 90 degress off nose
        #     actual: float = (
        #         self.closest_object.angle
        #         if self.closest_object.angle > 0
        #         else self.closest_object.angle + 2 * pi
        #     ) - radians(90)

        #     self.twist.angular.z = clamp(
        #         self.heading_PID.update(
        #             desired=0, 
        #             actual=-actual,
        #             dt=self.odom_dt), 
        #         -self.max_turn_rate, self.max_turn_rate
        #     )

        # # not evasive            
        # else:
        #     # get the desired heading
        #     desired_heading: float = Point(
        #         x=self.position.x,
        #         y=self.position.y,
        #         z=0
        #     ).heading_to(self.current_waypoint)

        #     # decide to turn left or right
        #     desired_heading = (
        #         desired_heading - 2 * pi
        #         if desired_heading - self.yaw > pi
        #         else desired_heading
        #     )
        #     self.twist.angular.z = clamp(
        #         self.heading_PID.update(
        #             desired=desired_heading, actual=self.yaw, dt=self.odom_dt
        #         ), -self.max_turn_rate, self.max_turn_rate
        #     )
        self.twist.linear.x = self.max_speed
        self.move()

    def on_lidar_callback(self) -> None:
        self.closest_object = min(
            self.detected_objects, key=lambda d_o: d_o.distance)

    def update(self) -> None:
        if self.last_callback == self.__odom_callback:
            self.on_odom_callback()

        elif self.last_callback == self.__lidar_callback:
            if self.detected_objects:
                self.on_lidar_callback()


def main() -> None:
    rclpy.init()

    print("Loading scenario...")
    scenario: Scenario = Scenario().loader(
    #     # "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/SearchAlgorithms/scenarios/AStar_15x15_bot-0o5_grid-1o0.json"
    #     "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/SearchAlgorithms/scenarios/RRT_15x15_bot-0o5_grid-1o0.json"
        "/home/thomas/ros2_ws/src/seagraves_unmanned_systems_pkg/SearchAlgorithms/scenarios/Dijkstra_10x10_bot-0o5_grid-1o0.json"
    )

    print("Finding path...")
    scenario.algorithm.find_path()

    print("Setting waypoints...")
    # Waypoints are stored in reverse order because the path finder creates the 
    # path starting at goal.
    waypoints: list[Waypoint] = []
    for node in scenario.algorithm.path:
        waypoints.append(Waypoint(x=node.x, y=node.y, radius=0.1))

    # waypoints: list[Waypoint] = [
    #     Waypoint(x=9, y=9, radius=0.1)
    # ]

    evader: Turtle = Turtle(
        heading_PID=PID(kp=4.5, ki=0.0, kd=0.25),
        throttle_PID=PID(kp=0.2, ki=0.0, kd=0.02),
        max_speed=0.95,
        max_turn_rate=2.84,
        waypoints=waypoints,
        namespace='turtle',
        name="Evader")

    evader.throttle_PID = None
    evader.max_speed = 0.15
    # evader.max_turn_rate = 1.0

    # for subscription in pursuer.subscriptions:
    #     if subscription.topic_name == f"{pursuer.name}/odom":
    #         pursuer.subscriptions.remove(subscription)
        
    #     elif subscription.topic_name == f"{pursuer.name}/clock":
    #         pursuer.subscriptions.remove(subscription)

    while rclpy.ok():
        rclpy.spin_once(evader)
        evader.update()

        if degrees(evader.roll) > 2:
            print(f"Too much roll, not enough rock...")
            break

        if evader.path_complete:
            print("Path complete")
            break

    evader.close_logs()
    evader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()