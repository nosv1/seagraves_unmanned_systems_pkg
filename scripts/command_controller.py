from __future__ import annotations

# python imports
from math import pi

# ros imports
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher

# personal imports
from support_module.Logger import Logger
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

class Waypoint:
    def __init__(
        self, 
        x: float, 
        y: float, 
        arrived_at_waypoint_raduis: float=0.1
    ):
        self.x = x
        self.y = y
        self.arrived_at_waypoint_radius = arrived_at_waypoint_raduis

class Controller(Node):
    def __init__(self, commands: list[Command] = None, waypoints: list[Waypoint]=None) -> None:
        super().__init__("CommandController")

        if commands:
            # knowing the start and stop_ns allows us to know when to switch to the next command
            # this assumes our commands are time based and not position based
            self.commands = commands
            self.commands[0].start_ns: int = self.get_clock().now().nanoseconds
            self.commands[0].stop_ns: int = self.commands[0].start_ns + (self.commands[0].duration * 1e9)

            self.heading_logger = Logger(headers=["time", "desired_heading", "actual_heading"], filename="heading_log.csv")
            self.command_logger = Logger(headers=["time", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"], filename="command_log.csv")
            
        elif waypoints:
            self.waypoints = waypoints

            self.pose_logger = Logger(headers=["time", "position_x", "position_y", "position_z", "roll", "pitch", "yaw"], filename="pose_log.csv")

        # initialize subscriber and publisher
        self.subscriber: Subscription = self.create_subscription(Odometry, "/odom", self.callback, 10)
        self.publisher: Publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # intialize the Twist() message
        self.msg: Twist = Twist()

        self.pid: PID = PID(kp=6.5, ki=0, kd=0)
        self.previous_time: int = self.get_clock().now().nanoseconds
        self.current_time: int = self.get_clock().now().nanoseconds
        self.dt: float = 0.0


    def publish(self) -> None:
        self.command_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            self.msg.linear.x, 
            self.msg.linear.y, 
            self.msg.linear.z, 
            self.msg.angular.x, 
            self.msg.angular.y, 
            self.msg.angular.z
        ])
        
        self.publisher.publish(self.msg)

    def callback(self, msg: Odometry) -> None:
        self.previous_time = self.current_time
        self.current_time = self.get_clock().now().nanoseconds
        self.dt = (self.current_time - self.previous_time) / 1e9
        
        position: Point = msg.pose.pose.position
        orientation: Quaternion = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        self.pose_logger.log([
            self.get_clock().now().nanoseconds / 1e9, 
            position.x, 
            position.y, 
            position.z, 
            self.roll * 180 / pi, 
            self.pitch * 180 / pi, 
            self.yaw * 180 / pi
        ])

        if self.commands:
            self.command_update()
        elif self.waypoints:
            self.waypoint_update()

    def command_update(self) -> None:

        # check if we need to switch to the next command
        self.commands[0].completed = self.commands[0].completed or self.commands[0].stop_ns < self.get_clock().now().nanoseconds
        self.commands[0].completed = self.commands[0].completed or (self.commands[0].use_pid and abs(self.pid.prev_error) < abs(self.commands[0].stop_error))
        if not self.commands[0].completed:

            if self.commands[0].use_pid:
                if self.commands[0].heading is not None:
                    self.msg = self.commands[0].execute(self.msg, stop=False)
                    # clamp pid output to max/min turnrate of the robot
                    self.msg.angular.z = max(
                        min(self.pid.update(self.commands[0].heading, self.yaw, self.dt), 2.84),
                        -2.84
                    )
                    if not self.commands[0].start_error:
                        self.commands[0].start_error = self.pid.prev_error
                        self.commands[0].stop_error = self.commands[0].start_error * 0.0001
                    self.heading_logger.log([self.get_clock().now().nanoseconds, self.commands[0].heading, self.yaw])
                    self.publish()

            # check if we've not published the command yet
            elif not self.commands[0].published:
                self.msg = self.commands[0].execute(self.msg)
                self.publish()
                self.commands[0].published = True

        # switch to the next command
        else:
            print(f'Switching to next command: {len(self.commands) - 1} remaining')
            # check if we need to 'revert' the command's values to 0
            if self.commands[0].stop_at_end:
                self.msg = self.commands[0].execute(self.msg, stop=True)
                self.publish()

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

def main():
    rclpy.init()

    ############################################################################

    # Commands \/\/\/

    problem_3_commands: list[Command] = [
        Command(duration=5, speed=1.5),
        Command(duration=2, radians=-.15, stop_at_end=True),
        Command(duration=5, speed=1.5, stop_at_end=True),
    ]

    problem_4_commands: list[Command] = [
        Command(duration=2, speed=.15),
        Command(duration=0, speed=.15, heading=90, stop_at_end=True),
    ]

    controller = Controller(problem_4_commands)

    # print("Starting command execution...")
    # while rclpy.ok() and command_controller.commands:
    #     rclpy.spin_once(command_controller)

    # print("Command execution complete!")

    # controller.heading_logger.close()
    # controller.command_logger.close()

    ############################################################################

    # Waypoints \/\/\/

    problem_5_waypoints: list[Waypoint] = [
        Waypoint(x=0, y=0),
        Waypoint(x=0, y=1),
        Waypoint(x=2, y=2),
        Waypoint(x=3, y=-3),
    ]

    print("Starting waypoint execution...")
    start_ns: int = controller.get_clock().now().nanoseconds
    while rclpy.ok() and controller.commands:
        rclpy.spin_once(controller)
    
    end_ns: int = controller.get_clock().now().nanoseconds
    print(f"Waypoint execution complete! Total time: {(end_ns - start_ns) / 1e9} seconds")

    controller.pose_logger.close()

    ############################################################################

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()