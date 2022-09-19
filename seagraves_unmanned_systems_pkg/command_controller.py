from __future__ import annotations

from geometry_msgs.msg import Point, Quaternion, Twist
import math
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple[float]:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class Command:
    def __init__(self, duration: float, speed: float=None, radians: float=None, stop_at_end: bool=False):
        self.duration = duration
        self.speed = speed
        self.radians = radians
        self.stop_at_end = stop_at_end

        self.start_ns: int = None
        self.stop_ns: int = None
        self.published: bool = False

    def execute(self, msg: Twist, stop=False) -> Twist:
        """
        Updates the msg with the command's values or 0.0 if stop is True
        """
        if self.speed is not None:
            msg.linear.x = self.speed if not stop else 0.0

        if self.radians is not None:
            msg.angular.z = self.radians if not stop else 0.0
        return msg

class CommandController(Node):
    def __init__(self, commands: list[Command]) -> None:
        super().__init__("CommandController")

        # knowing the start and stop_ns allows us to know when to switch to the next command
        # this assumes our commands are time based and not position based
        self.commands = commands
        self.commands[0].start_ns = self.get_clock().now().nanoseconds
        self.commands[0].stop_ns = self.commands[0].start_ns + (self.commands[0].duration * 1e9)

        # initialize subscriber and publisher
        self.subscriber = self.create_subscription(Odometry, "/odom", self.callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # intialize the Twist() message
        self.msg = Twist()

    def publish(self) -> None:
        self.get_logger().info(f"Publishing Linear: [{self.msg.linear.x:.2f}, {self.msg.linear.y:.2f}, {self.msg.linear.z:.2f}]")
        self.get_logger().info(f"Publishing Angular: [{self.msg.angular.x:.2f}, {self.msg.angular.y:.2f}, {self.msg.angular.z:.2f}]")
        
        self.publisher.publish(self.msg)
        rclpy.spin_once(self, timeout_sec=0.1)

    def callback(self, msg: Odometry) -> None:
        position: Point = msg.pose.pose.position
        orientation: Quaternion = msg.pose.pose.orientation
        rotation: tuple[float] = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        self.get_logger().info(f"Position: [{position.x:.2f}, {position.y:.2f}, {position.z:.2f}]")
        self.get_logger().info(f"Rotation: [{rotation[0]:.2f}, {rotation[1]:.2f}, {rotation[2]:.2f}]")

        # check if we need to switch to the next command
        if self.commands[0].stop_ns > self.get_clock().now().nanoseconds:
            # check if we've not published the command yet
            if not self.commands[0].published:
                self.msg = self.commands[0].execute(self.msg)
                self.publish()
                self.commands[0].published = True

        # switch to the next command
        else:
            # check if we need to 'revert' the command's values to 0
            if self.commands[0].stop_at_end:
                self.msg = self.commands[0].execute(self.msg, stop=True)
                self.publish()

            # pop the current command and set the next command's start_ns and stop_ns
            if (len(self.commands) > 1):
                self.commands.pop(0)
                self.commands[0].start_ns = self.get_clock().now().nanoseconds
                self.commands[0].stop_ns = self.commands[0].start_ns + (self.commands[0].duration * 1e9)

            else:
                self.commands.pop(0)

def main():
    rclpy.init()

    commands: list[Command] = [
        Command(duration=5, speed=1.5),
        Command(duration=2, radians=-.15, stop_at_end=True),
        Command(duration=5, speed=1.5, stop_at_end=True),
    ]

    pid = CommandController(commands)

    # check if pid is alive
    while pid.commands:
        rclpy.spin_once(pid)

    rclpy.shutdown()

if __name__ == "__main__":
    main()