#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file pose_relay.py
@brief Down-samples a high-rate pose source and republishes PoseStamped and PointStamped.

@details
This node listens to a pose source (typically `/odometry`) and periodically
republishes:
- `PoseStamped` at `output_topic` (default: `/drone/pose_1hz`)
- `PointStamped` at `point_topic`   (default: `/drone/position`)

It is useful for UIs or other nodes that only need a low-rate pose feed.

@par Parameters
- `source_topic`  (string, default: "/odometry")
    Topic to subscribe to; can be an Odometry or PoseStamped stream.
- `source_type`   (string, default: "odom")
    Either `"odom"` or `"pose"`. If `"odom"`, messages must be `nav_msgs/Odometry`.
    If `"pose"`, messages must be `geometry_msgs/PoseStamped`.
- `output_topic`  (string, default: "/drone/pose_1hz")
    Output topic for the down-sampled `geometry_msgs/PoseStamped`.
- `point_topic`   (string, default: "/drone/position")
    Output topic for the down-sampled `geometry_msgs/PointStamped`.
- `rate_hz`       (double, default: 1.0)
    Publication rate for down-sampled outputs.

@par Subscriptions
- `/odometry` or another topic per `source_topic` (Odometry or PoseStamped)

@par Publications
- `/drone/pose_1hz` (`geometry_msgs/PoseStamped`)
- `/drone/position` (`geometry_msgs/PointStamped`)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry


class PoseRelay(Node):
    """Relay node that converts a fast pose source to low-rate Pose/Point topics."""

    def __init__(self) -> None:
        super().__init__('pose_relay')

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter('source_topic', '/odometry')
        self.declare_parameter('source_type', 'odom')  # "odom" or "pose"
        self.declare_parameter('output_topic', '/drone/pose_1hz')
        self.declare_parameter('point_topic', '/drone/position')
        self.declare_parameter('rate_hz', 1.0)

        self._source_topic: str = self.get_parameter('source_topic').get_parameter_value().string_value
        self._source_type: str = self.get_parameter('source_type').get_parameter_value().string_value.lower()
        self._output_topic: str = self.get_parameter('output_topic').get_parameter_value().string_value
        self._point_topic: str = self.get_parameter('point_topic').get_parameter_value().string_value
        self._rate_hz: float = float(self.get_parameter('rate_hz').get_parameter_value().double_value)

        if self._source_type not in ('odom', 'pose'):
            self.get_logger().warn(f'Unknown source_type "{self._source_type}", falling back to "odom".')
            self._source_type = 'odom'

        # ---------------------------
        # QoS
        # ---------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---------------------------
        # Publishers
        # ---------------------------
        self.pub_pose = self.create_publisher(PoseStamped, self._output_topic, qos)
        self.pub_point = self.create_publisher(PointStamped, self._point_topic, qos)

        # ---------------------------
        # Subscriptions
        # ---------------------------
        if self._source_type == 'odom':
            self.sub = self.create_subscription(Odometry, self._source_topic, self._on_odom, qos)
        else:
            self.sub = self.create_subscription(PoseStamped, self._source_topic, self._on_pose_in, qos)

        # ---------------------------
        # State and timer
        # ---------------------------
        self._latest_pose: PoseStamped | None = None
        timer_period = 1.0 / max(1e-3, self._rate_hz)
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f'PoseRelay: src=({self._source_type}) {self._source_topic} → '
            f'{self._output_topic} & {self._point_topic} @ {self._rate_hz:.3f} Hz'
        )

    # ---------------------------
    # Callbacks
    # ---------------------------
    def _on_odom(self, msg: Odometry) -> None:
        """Convert incoming Odometry to PoseStamped and cache it."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose

    def _on_pose_in(self, msg: PoseStamped) -> None:
        """Cache incoming PoseStamped directly."""
        self._latest_pose = msg

    def _on_timer(self) -> None:
        """Publish the latest pose and point at the configured rate."""
        if self._latest_pose is None:
            return

        # PoseStamped (unchanged)
        self.pub_pose.publish(self._latest_pose)

        # PointStamped (position only)
        pt = PointStamped()
        pt.header = self._latest_pose.header
        pt.point = self._latest_pose.pose.position
        self.pub_point.publish(pt)


def main() -> None:
    """Entry point."""
    rclpy.init()
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()