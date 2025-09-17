#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Twist

GUI_TO_CMD = {
    "HALT": "halt",
    "MOVE TO GOAL": "move_to_goal",
    "LAND": "land",
    "TAKEOFF": "takeoff",
    "EMERGENCY LAND": "emergency_land",
}

STATUSES = [
    "Pre Flight Checks",
    "Landed",
    "Halted",
    "Arrived at Goal",

    "Taking off",
    "Landing",
    "Halting",
    "Moving to Goal",
    "Emergency Landing",
]


class FlightControl(Node):
    def __init__(self):
        super().__init__('flight_control')
        self.get_logger().info("Flight Control Node Started")

        # --- Publishers ---
        self.pub_status  = self.create_publisher(String, '/movement/status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)  # not used yet, reserved

        # --- Subscribers (listen to GUI + pose) ---
        self.sub_cmd   = self.create_subscription(String, '/cmd/control', self.on_cmd, 10)
        self.sub_goal  = self.create_subscription(PointStamped, '/cmd/goal', self.on_goal, 10)

        # use the callbacks you actually defined below
        self.sub_height = self.create_subscription(Float32, '/cmd/height', self.sub_height, 10)
        self.sub_pose   = self.create_subscription(PoseStamped, '/drone/pose_1hz', self.sub_pose, 10)

        # --- Internal state ---
        self.cmd = 'pre-flight'
        self.goal_xyz = None
        self.target_height = None
        self._last_status = None
        self.current_pose = None

        # main loop (just relays status for now)
        self.timer = self.create_timer(0.2, self.main_loop)  # 5 Hz is fine for status relay

        # publish initial status so GUI leaves "CONNECTING..."

        self.currentX = None
        self.currentY = None
        self.currentZ = None

        self.preFlightChecks = False

        self.heightTolerance = 0.5

        self.kp_z = 1.2
        self.max_up_speed = 2.0

        self.lastCommand = None
        self.takeOffSet = False
        self.goalSet = False
        self.preFlightChecks = True   # or False; set how you need

        self.status = "Pre Flight Checks"

    # ----------------- Callbacks -----------------
    def on_cmd(self, msg: String):
        text = msg.data.strip().upper()
        self.get_logger().info(f"/cmd/control: {text}")
        if text in GUI_TO_CMD:
            self.cmd = GUI_TO_CMD[text]
        else:
            self.cmd = text.lower()
        self.lastCommand = self.cmd
        # optional immediate status switch for clarity:
        if self.cmd == 'takeoff':
            self.set_status('Taking off')
        elif self.cmd == 'land':
            self.set_status('Landing')
        elif self.cmd == 'halt':
            self.set_status('Halted')
        elif self.cmd == 'move_to_goal':
            self.set_status('Moving to Goal')
        elif self.cmd == 'emergency_land':
            self.set_status('Emergency Landing')


    def on_goal(self, msg: PointStamped):
        self.goal_xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        self.get_logger().info(f"/cmd/goal: {self.goal_xyz}")

    def on_height(self, msg: Float32):
        self.target_height = float(msg.data)
        self.get_logger().info(f"/cmd/height: {self.target_height:.2f} m")

    def on_pose(self, msg: PoseStamped):
        self.current_pose = msg.pose
        # keep logs light; flip to info if you want to see it
        self.get_logger().debug(
            f"pose_1hz x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} z={msg.pose.position.z:.2f}"
        )

    # ----------------- State relay only -----------------
    def main_loop(self):
        # Map command keyword -> outward-facing status expected by GUI
        if self.status == 'Pre Flight Checks':
            self.pre_flight_checks()
        elif self.status == 'Landed':
            self.landed()
        elif self.status == 'Halted':
            self.halted()
        elif self.status == 'Arrived at Goal':
            self.arrived_at_goal()

        elif self.status == 'Taking off':
            self.taking_off()
        elif self.status == 'Landing':
            self.landing()
        elif self.status == 'Halting':
            self.halting()
        elif self.status == 'Moving to Goal':
            self.moving_to_goal()
        elif self.status == 'Emergency Landing':
            self.emergency_landing()
        else:
            # Unknown/idle: stay in pre-flight to avoid confusion
            self.set_status("Pre Flight Checks")
                        
    def pre_flight_checks(self):
        self.set_status("Pre Flight Checks")
        if self.preFlightChecks:
            self.set_status("Landed")
        else:
            self.goalSet = False
            self.takeOffSet = False
            self.height_difference = None
            self.targetHeight = 5
            self.set_status("Landed")

    def landed(self):
        if self.lastCommand == 'takeoff':
            self.set_status('Taking off')
        elif self.lastCommand == 'move_to_goal':
            self.set_status('Taking off')
        else:
            return

    def halted(self):
        self.set_status("Halted")
        if self.takeOffSet:
            self.set_status("Taking off")
        else:
            return
        
    def arrived_at_goal(self):
        self.set_status("Arrived at Goal")

    def taking_off(self):
        # Need pose and Z
        if self.currentZ is None:
            self.zero_twist()
            return

        # choose target height (GUI-set or default 2.0 m)
        tgt = self.target_height if self.target_height is not None else 2.0

        # error = target - current (positive means 'go up')
        z_err = float(tgt - self.currentZ)

        # within tolerance? stop and switch to Halted
        if abs(z_err) <= self.heightTolerance:
            self.zero_twist()
            self.set_status("Halted")
            return

        # proportional control on Z, only ascend in takeoff
        vz = self.kp_z * z_err
        if vz < 0.0:
            vz = 0.0
        if vz > self.max_up_speed:
            vz = self.max_up_speed

        cmd = Twist()
        cmd.linear.z = vz
        self.pub_cmd_vel.publish(cmd)

    def landing(self):
        self.set_status("Landing")
    
    def halting(self):
        self.set_status("Halting")

    def moving_to_goal(self):
        self.set_status("Moving to Goal")

    def emergency_landing(self):
        self.set_status("Emergency Landing")

    # ----------------- Helpers -----------------
    def set_status(self, status: str):
        if status not in STATUSES:
            self.get_logger().error(f"Invalid status: {status}")
            return
        if getattr(self, "status", None) == status:
            return
        self.status = status                      # <-- keep an internal state string
        self._last_status = status
        self.pub_status.publish(String(data=status))
        self.get_logger().info(f"Status: {status}")


    def setHeight(self, height: float):
        """Setter for target height in meters."""
        self.target_height = height
        self.get_logger().info(f"Target height updated via setHeight: {self.target_height:.2f} m")

    def sub_height(self, msg: Float32):
        """Subscriber callback for /cmd/height topic."""
        self.setHeight(float(msg.data))

    def sub_pose(self, msg: PoseStamped):
        """Update currentX/Y/Z from /drone/pose_1hz."""
        self.currentX = float(msg.pose.position.x)
        self.currentY = float(msg.pose.position.y)
        self.currentZ = float(msg.pose.position.z)
        # Optional debug:
        # self.get_logger().debug(f"pose: x={self.currentX:.2f} y={self.currentY:.2f} z={self.currentZ:.2f}")

    def zero_twist(self):
        self.pub_cmd_vel.publish(Twist())

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))


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
