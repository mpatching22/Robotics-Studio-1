#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped

COMMANDS = [
    "pre-flight", 
    "move_to_goal", 
    "halt", 
    "takeoff",
    "land",
    "emergency_land"
]

STATUSES = [
    "Pre Flight Checks",
    "Landed",
    "Landing",
    "Taking off",
    "Halted",
    "Moving to goal",
    "Arrived at goal",
]


class FlightControl(Node):
    def __init__(self):
        super().__init__('flight_control')

        self.get_logger().info("Flight Control Node Started")

        # Publisher for velocity commands
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Status publisher
        self.pub_status = self.create_publisher(String, "/movement/status", 10)

        # Goal-related subscribers
        self.sub_goal_position = self.create_subscription(
            PointStamped, "/goal/position", self.on_goal_position, 10
        )
        self.sub_goal_distance = self.create_subscription(
            Float32, "/goal/distance", self.on_goal_distance, 10
        )
        self.sub_goal_time = self.create_subscription(
            Float32, "/goal/time", self.on_goal_time, 10
        )

        # Internal goal state (optional, handy later)
        self.goal_position = None      # will hold (x, y, z)
        self.goal_distance = None      # metres
        self.goal_time = None          # seconds

        self.cmd = 'pre-flight' 

        self.timer = self.create_timer(0.1, self.main_loop)  # 10 Hz

    def main_loop(self):
        if self.cmd == 'pre-flight':
            self.preFlight()

        elif self.cmd == 'emergency_land':
            self.emergency_land()

        elif self.cmd == 'halt':
            self.halt()

        elif self.cmd == 'land':
            self.land()

        elif self.cmd == 'takeoff':
            self.takeoff()

    def preFlight(self):
        self.set_status("Pre Flight Checks")
        self.get_logger().info("Pre-flight checks complete.")
        
    def emergency_land(self):
        self.set_status("Emergency Landing")
        self.get_logger().info("Emergency landing initiated!")

    def halt(self):
        self.set_status("Halted")
        self.get_logger().info("Halting all movements.")

    def land(self):
        self.set_status("Landing")
        self.get_logger().info("Landing sequence initiated.")

    def takeoff(self):
        self.set_status("Taking off")
        self.get_logger().info("Takeoff sequence initiated.")

    def set_status(self, status: str) -> None:
        if status not in STATUSES:
            self.get_logger().error(f"Invalid status: {status}")
            return
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)
        self.get_logger().info(f"Status: {status}")
        
def main(args=None):
    rclpy.init(args=args)

    node = FlightControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
