#!usr/bin/env python3

from __future__ import annotations

# python imports
from math import atan2, pi
from os import system
import random

# ros imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rosgraph_msgs.msg import Clock

# personal imports
from support_module.GeneticAlgorithm import GeneticAlgorithm
from support_module.Logger import Logger
from support_module.math_tools import clamp
from support_module.PID import PID
from support_module.quaternion_tools import euler_from_quaternion

class Command:
    def __init__(self, duration: float, speed: float=None, radians: float=None, heading: float=None, stop_at_end: bool=False):
        """
        Set duration to 0 to use a PID controller
        :param duration: time in seconds to execute the command
        :param speed: speed in m/s
        :param radians: radians per second
        :param heading: heading in degrees (0-360)
        """
        self.duration = duration        # if 0 then use PID
        self.speed = speed              # desired m/s
        self.radians = radians          # desired rad/s
        self.heading = heading * pi / 180.0 if heading is not None else heading
        self.stop_at_end = stop_at_end  # revert to 0 at the end of the command

        self.use_pid: bool = self.duration == 0
        self.start_error: float = 0
        self.stop_error: float = 0

        self.start_ns: int = None
        self.stop_ns: int = None

        self.published: bool = False
        self.completed: bool = False

    def execute(self, msg: Twist, stop=False) -> Twist:
        """
        Updates the msg with the command's values or 0.0 if stop is True
        """
        if self.speed is not None:
            msg.linear.x = self.speed if not stop else 0.0

        if self.radians is not None or self.heading is not None:
            if self.radians is not None:
                msg.angular.z = self.radians if not stop else 0.0
            elif self.heading is not None:
                msg.angular.z = msg.angular.z if not stop else 0.0
        return msg

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"

    def distance_to(self, other: Point) -> float:
        """
        Euclidean distance between two points
        """
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def heading_to(self, other: Point) -> float:
        """
        Heading from self to other in radians
        """
        return atan2(other.y - self.y, other.x - self.x)

class Waypoint(Point):
    def __init__(self, arrived_at_waypoint_radius: float=0.1, **kwargs):
        super().__init__(**kwargs)
        self.arrived_at_waypoint_radius = arrived_at_waypoint_radius

    def __str__(self) -> str:
        return super().__str__()

    def arrived_at_waypoint(self, other: Point) -> bool:
        return self.distance_to(other) <= self.arrived_at_waypoint_radius

class Controller(Node):
    def __init__(
        self, 
        commands: list[Command] = None, 
        waypoints: list[Waypoint]=None,
        heading_pid: PID=None,
        throttle_pid: PID=None,
    ) -> None:
        super().__init__("CommandController")

        self.commands = commands
        self.waypoints = waypoints
        self.heading_pid = heading_pid
        self.throttle_pid = throttle_pid

        # initialize subscriber and publisher
        self.odom_subscriber: Subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.clock_subscriber: Subscription = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.cmd_vel_publisher: Publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_publisher: Publisher = self.create_publisher(Odometry, "/odom", 10)

        # intialize the Twist() message
        self.twist_msg: Twist = Twist()
        self.odom_msg: Odometry = Odometry()

        self.previous_time: int = self.get_clock().now().nanoseconds
        self.current_time: int = self.get_clock().now().nanoseconds
        self.dt: float = 0.0

        if commands:
            # knowing the start and stop_ns allows us to know when to switch to the next command
            # this assumes our commands are time based and not position based
            self.commands[0].start_ns: int = self.get_clock().now().nanoseconds
            self.commands[0].stop_ns: int = self.commands[0].start_ns + (self.commands[0].duration * 1e9)

            self.heading_logger = Logger(headers=["time", "desired_heading", "actual_heading"], filename="heading_log.csv")
            
        elif waypoints:
            self.twist_msg.linear.x = .95
            self.current_waypoint_number: int = 1
            self.average_speed_per_waypoint: float = 0  # avg (waypoint-to-waypoint-distance / time)
            self.last_waypoint_reached_at: int = self.current_time  # ns

        self.command_logger = Logger(headers=["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"], filename="command_log.csv")
        self.pose_logger = Logger(headers=["time", "position_x", "position_y", "position_z", "roll", "pitch", "yaw"], filename="pose_log.csv")

        self.sim_start: float = 0
        self.sim_current: float = 0
        self.sim_ellapsed: float = 0

    def publish_cmd_vel(self) -> None:
        self.command_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.twist_msg.linear.x, 
            self.twist_msg.linear.y, 
            self.twist_msg.linear.z, 
            self.twist_msg.angular.x, 
            self.twist_msg.angular.y, 
            self.twist_msg.angular.z
        ])
        
        self.cmd_vel_publisher.publish(self.twist_msg)

    def publish_odom(self) -> None:
        self.odom_publisher.publish(self.odom_msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.previous_time = self.current_time
        self.current_time = self.get_clock().now().nanoseconds
        self.dt = (self.current_time - self.previous_time) / 1e9
        
        self.position: Point = msg.pose.pose.position
        self.orientation: Quaternion = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w
        )

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.position.x, 
            self.position.y, 
            self.position.z, 
            self.roll * 180 / pi, 
            self.pitch * 180 / pi, 
            self.yaw * 180 / pi
        ])

        if self.commands:
            self.command_update()
        elif self.waypoints:
            self.waypoint_update()

    def clock_callback(self, msg: Clock) -> None:
        self.sim_start = msg.clock.sec + msg.clock.nanosec / 1e9 if not self.sim_start else self.sim_start
        self.sim_current = msg.clock.sec + msg.clock.nanosec / 1e9
        self.sim_ellapsed = self.sim_current - self.sim_start

    def command_update(self) -> None:

        # check if we need to switch to the next command
        self.commands[0].completed = self.commands[0].completed or self.commands[0].stop_ns < self.get_clock().now().nanoseconds
        self.commands[0].completed = self.commands[0].completed or (self.commands[0].use_pid and abs(self.heading_pid.prev_error) < abs(self.commands[0].stop_error))
        if not self.commands[0].completed:

            if self.commands[0].use_pid:
                if self.commands[0].heading is not None:
                    self.twist_msg = self.commands[0].execute(self.twist_msg, stop=False)
                    # clamp pid output to max/min turnrate of the robot
                    self.twist_msg.angular.z = clamp(
                        self.heading_pid.update(self.commands[0].heading, self.yaw, self.dt), -2.84, 2.84
                    )
                    if not self.commands[0].start_error:
                        self.commands[0].start_error = self.heading_pid.prev_error
                        self.commands[0].stop_error = self.commands[0].start_error * 0.0001
                    self.heading_logger.log([self.get_clock().now().nanoseconds, self.commands[0].heading, self.yaw])
                    self.publish_cmd_vel()

            # check if we've not published the command yet
            elif not self.commands[0].published:
                self.twist_msg = self.commands[0].execute(self.twist_msg)
                self.publish_cmd_vel()
                self.commands[0].published = True

        # switch to the next command
        else:
            print(f'Switching to next command: {len(self.commands) - 1} remaining')
            # check if we need to 'revert' the command's values to 0
            if self.commands[0].stop_at_end:
                self.twist_msg = self.commands[0].execute(self.twist_msg, stop=True)
                self.publish_cmd_vel()

            # pop the current command and set the next command's start_ns and stop_ns
            if (len(self.commands) > 1):
                self.commands.pop(0)
                self.commands[0].start_ns = self.get_clock().now().nanoseconds
                if self.commands[0].duration > 0:
                    self.commands[0].stop_ns = self.commands[0].start_ns + (self.commands[0].duration * 1e9)
                else:
                    self.commands[0].stop_ns = float("inf")

            else:
                self.commands.pop(0)

    def waypoint_update(self) -> None:
        current_position: Point = Point(
            x=self.position.x,
            y=self.position.y
        )            

        acutal_heading: float = self.yaw
        desired_heading: float = current_position.heading_to(self.waypoints[self.current_waypoint_number])
        pid_desired_heading: float = (
            desired_heading - 2 * pi
            if desired_heading - acutal_heading > pi
            else desired_heading
        )
        self.twist_msg.angular.z = clamp(
            self.heading_pid.update(
                pid_desired_heading, self.yaw, self.dt
            ), -2.84, 2.84
        )
        # self.twist_msg.linear.x = clamp(
        #     -self.throttle_pid.update(
        #         pid_desired_heading, self.yaw, self.dt
        #     ),
        #     -2.0, 0.0
        # ) + 2
        
        # try to switch to the next waypoint
        if self.waypoints[self.current_waypoint_number].arrived_at_waypoint(current_position):
            print(f"Arrived at waypoint: {self.waypoints[self.current_waypoint_number]}")
            self.current_time: int = self.get_clock().now().nanoseconds

            waypoint_speed = self.waypoints[self.current_waypoint_number].distance_to(
                self.waypoints[self.current_waypoint_number - 1]
            ) / ((self.current_time - self.last_waypoint_reached_at) / 1e9)
            # (average-speed * waypoints-reached + waypoint_speed) / (new-waypoints-reached + 1)
            self.average_speed_per_waypoint = (
                (self.average_speed_per_waypoint * (self.current_waypoint_number - 1) + waypoint_speed) / 
                (self.current_waypoint_number)
            )
            self.last_waypoint_reached_at = self.current_time
            self.current_waypoint_number += 1
            
            if self.current_waypoint_number == len(self.waypoints):
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
            else:
                print(f"Next waypoint: {self.waypoints[self.current_waypoint_number]}")
            
        self.publish_cmd_vel()

def main():
    rclpy.init()

    ############################################################################

    # Commands \/\/\/

    # problem_3_commands: list[Command] = [
    #     Command(duration=5, speed=1.5),
    #     Command(duration=2, radians=-.15, stop_at_end=True),
    #     Command(duration=5, speed=1.5, stop_at_end=True),
    # ]

    # problem_4_commands: list[Command] = [
    #     Command(duration=2, speed=.15),
    #     Command(duration=0, speed=.15, heading=90, stop_at_end=True),
    # ]

    # controller = Controller(commands=problem_4_commands)

    # print("Starting command execution...")
    # while rclpy.ok() and controller.commands:
    #     rclpy.spin_once(controller)

    # print("Command execution complete!")

    # controller.heading_logger.close()

    ############################################################################

    # Waypoints \/\/\/

    problem_5_waypoints: list[Waypoint] = [
        Waypoint(x=0, y=0),
        Waypoint(x=0, y=1),
        Waypoint(x=2, y=2),
        Waypoint(x=3, y=-3),
    ]
    # heading_pid: PID = PID(kp=4.5, ki=0, kd=0.25)
    # throttle_pid: PID = PID(kp=0.5, ki=0, kd=0.25)
    max_iteration_time = 15.0

    genetic_algorithm: GeneticAlgorithm = GeneticAlgorithm(
        population_size=8,
        gene_randomizers=[
            (random.random, (3, 6)),   # heading_Kp
            (random.random, (-1, 1)),  # heading_Ki 
            (random.random, (-1, 1)),  # heading_Kd 
            (random.random, (3, 6)),   # throttle_Kp
            (random.random, (-1, 1)),  # throttle_Ki
            (random.random, (-1, 1))   # throttle_Kd
        ],
        mutation_std=0.1,
        reverse_population_sort=True
    )

    generations = 100
    i = 0
    while i <= generations:
        print(f"Generation {i+1}/{generations}")

        for i, choromosome in enumerate(genetic_algorithm.population):
            print(f"Chromosome {i+1}/{genetic_algorithm.population_size}")
            print("Resetting world...")
            system("ros2 service call /reset_world std_srvs/srv/Empty")

            controller = Controller(
                waypoints=problem_5_waypoints.copy(),
                heading_pid=PID(kp=choromosome.genes[0].gene, ki=choromosome.genes[1].gene, kd=choromosome.genes[2].gene),
                # throttle_pid=PID(kp=choromosome.genes[3].gene, ki=choromosome.genes[4].gene, kd=choromosome.genes[5].gene),
            )

            print("Starting waypoint execution...")
            print(f"Heading: {controller.heading_pid}")
            # print(f"Throttle: {controller.throttle_pid}")
            # start_ns: int = controller.get_clock().now().nanoseconds
            while (
                rclpy.ok() and 
                controller.current_waypoint_number < len(controller.waypoints) and
                controller.sim_ellapsed < max_iteration_time
            ):
                rclpy.spin_once(controller)
            # end_ns: int = controller.get_clock().now().nanoseconds

            print("Waypoint execution complete!")
            # print(f"Total IRL Time: {(end_ns - start_ns) / 1e9} seconds")
            print(f"Total Sim Time: {controller.sim_ellapsed:.3f} seconds")
            print(f"Average Speed: {controller.average_speed_per_waypoint:.3f} m/s")

            controller.twist_msg.linear.x = 0.0
            controller.twist_msg.angular.z = 0.0
            controller.publish_cmd_vel()
            controller.destroy_node()

            if controller.current_waypoint_number > 1:
                choromosome.fitness = controller.average_speed_per_waypoint
            else:
                choromosome.fitness = float('-inf')

        genetic_algorithm.mutate_population(percent=.20)

        i += 1

    ############################################################################

    controller.command_logger.close()
    controller.pose_logger.close()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()