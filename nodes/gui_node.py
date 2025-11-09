#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file gui_node.py
@brief Trailblazer operator GUI (PySide6) with ROS 2 integration.

@details
Two-pane control and status panel used to:
- Send goals, height setpoints, and control verbs to the flight controller.
- Display current status, pose, HAG (height-above-ground), distance/time to goal.
- Preview a camera stream (optional, via cv_bridge) and render a simple map with the
  current XY position.
- Generate map/graph images from CSV logs using `trailblazer_utils.map_printer`.

@par Publications
- `/cmd/control` (`std_msgs/String`)                : Free-form command text (e.g., "MOVE TO GOAL", "HOVER").
- `/cmd/goal`    (`geometry_msgs/PointStamped`)     : Goal in `map` frame (Z=0, Nav2 handles XY).
- `/cmd/height`  (`std_msgs/Float32`)               : Target HAG in metres.

@par Subscriptions
- `/movement/status` (`std_msgs/String`)            : Human-readable state from flight_control.
- `/drone/pose_1hz`  (`geometry_msgs/PoseStamped`)  : Down-sampled pose for UI.
- `/altitude/hag`    (`std_msgs/Float32`)           : Current HAG.
- `/goal/distance`   (`std_msgs/Float32`)           : Remaining distance to goal (m).
- `/goal/time`       (`std_msgs/Float32`)           : Time metric (ETA/elapsed) in seconds.
- `/camera/image`    (`sensor_msgs/Image`, optional): RGB/BGR camera preview (requires cv_bridge).

@par Notes
- cv_bridge is imported lazily to avoid NumPy/ABI issues; if unavailable, the GUI disables camera preview.
- The ROS event loop is integrated with Qt using `QTimer` + `rclpy.spin_once`.
"""

from __future__ import annotations

import sys
import signal
from pathlib import Path
from typing import Optional

# ------------------------------
# Qt
# ------------------------------
from PySide6.QtCore import Qt, QSize, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
from PySide6.QtWidgets import (
    QApplication, QWidget, QDialog, QLabel, QHBoxLayout, QVBoxLayout,
    QFrame, QPushButton, QSizePolicy, QListWidget,
    QFileDialog, QMessageBox, QLineEdit
)

# ------------------------------
# ROS 2
# ------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped


# =============================================================================
# ROS node backend
# =============================================================================
class GuiNode(Node):
    """
    @class GuiNode
    @brief ROS 2 backend for the GUI (publishers, subscribers, and bridge to Qt).

    @details
    Exposes convenience publishing methods (`send`, `publish_goal`, `publish_height`)
    and forwards subscribed messages to the GUI via Qt signals stored on a back-reference
    (`self.gui_ref`), which is set by the Qt widget after construction.
    """

    def __init__(self) -> None:
        """
        @brief Construct publishers/subscribers and attempt to enable the camera.
        """
        super().__init__('gui_node')

        # ---- Publishers ----
        self.pub_cmd = self.create_publisher(String, '/cmd/control', 10)
        self.pub_goal = self.create_publisher(PointStamped, '/cmd/goal', 10)
        self.pub_height = self.create_publisher(Float32, '/cmd/height', 10)

        # ---- Subscribers ----
        self.create_subscription(String, '/movement/status', self.status_callback, 10)
        self.create_subscription(PoseStamped, '/drone/pose_1hz', self.pose_callback_ps, 10)
        self.create_subscription(Float32, '/altitude/hag', self.hag_cb, 10)
        self.create_subscription(Float32, '/goal/distance', self.goal_dist_cb, 10)
        self.create_subscription(Float32, '/goal/time', self.goal_time_cb, 10)

        # Optional camera: lazy import to avoid cv_bridge/NumPy issues
        self.sub_cam = None
        self.bridge = None
        try:
            from cv_bridge import CvBridge as _CvBridge  # type: ignore
            import cv2 as _cv2  # noqa: F401  (kept for potential future operations)
            self.bridge = _CvBridge()
            self.sub_cam = self.create_subscription(Image, '/camera/image', self.camera_cb, 10)
            self.get_logger().info('cv_bridge detected: camera preview enabled')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge not available; camera disabled: {e}')

        # Back-reference set by TwoPaneGUI
        self.gui_ref: Optional[TwoPaneGUI] = None

    # -------------------------------------------------------------------------
    # Subscriber callbacks → emit Qt signals via gui_ref
    # -------------------------------------------------------------------------
    def status_callback(self, msg: String) -> None:
        """
        @brief Forward status text to GUI.
        @param msg Incoming status string.
        """
        if self.gui_ref:
            self.gui_ref.status_signal.emit(msg.data)

    def goal_dist_cb(self, msg: Float32) -> None:
        """
        @brief Forward distance-to-goal to GUI.
        @param msg Remaining distance (metres).
        """
        if self.gui_ref:
            self.gui_ref.goal_dist_signal.emit(float(msg.data))

    def goal_time_cb(self, msg: Float32) -> None:
        """
        @brief Forward time metric (ETA/elapsed) to GUI.
        @param msg Time in seconds.
        """
        if self.gui_ref:
            self.gui_ref.goal_eta_signal.emit(float(msg.data))

    def hag_cb(self, msg: Float32) -> None:
        """
        @brief Forward current HAG to GUI.
        @param msg Height-above-ground (metres).
        """
        if self.gui_ref:
            self.gui_ref.hag_signal.emit(float(msg.data))

    def pose_callback_ps(self, msg: PoseStamped) -> None:
        """
        @brief Forward pose (PoseStamped) to GUI as (x, y, z).
        @param msg geometry_msgs/PoseStamped
        """
        if self.gui_ref:
            p = msg.pose.position
            self.gui_ref.pose_signal.emit(float(p.x), float(p.y), float(p.z))

    def pose_callback_pcs(self, msg: PoseWithCovarianceStamped) -> None:
        """
        @brief Alternate pose callback (PoseWithCovarianceStamped).
        @param msg geometry_msgs/PoseWithCovarianceStamped
        @note Not wired by default, but retained for compatibility.
        """
        if self.gui_ref:
            p = msg.pose.pose.position
            self.gui_ref.pose_signal.emit(float(p.x), float(p.y), float(p.z))

    def camera_cb(self, msg: Image) -> None:
        """
        @brief Convert ROS Image → QImage (via cv_bridge) and emit to GUI.
        @param msg sensor_msgs/Image
        @note Exits early if cv_bridge is unavailable.
        """
        try:
            if not self.bridge or not self.gui_ref:
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.gui_ref.camera_signal.emit(frame)
        except Exception as e:
            self.get_logger().warn(f'Camera conversion failed: {e}')

    # -------------------------------------------------------------------------
    # Convenience publishers (called by the GUI)
    # -------------------------------------------------------------------------
    def send(self, msg_text: str) -> None:
        """
        @brief Publish a free-form control command on `/cmd/control`.
        @param msg_text Command text (e.g. "MOVE TO GOAL", "HOVER").
        """
        msg_text = (msg_text or '').strip()
        if not msg_text:
            return
        self.pub_cmd.publish(String(data=msg_text))
        self.get_logger().info(f"Published: {msg_text}")

    def publish_goal(self, x: float, y: float) -> None:
        """
        @brief Publish a goal on `/cmd/goal` (map frame, Z=0).
        @param x Goal X (m)
        @param y Goal Y (m)
        """
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), 0.0
        self.pub_goal.publish(msg)
        self.get_logger().info(f"Published /cmd/goal: ({x}, {y})")

    def publish_height(self, h: float) -> None:
        """
        @brief Publish a height setpoint on `/cmd/height`.
        @param h Target HAG in metres.
        """
        self.pub_height.publish(Float32(data=float(h)))
        self.get_logger().info(f"Published /cmd/height: {h}")


# =============================================================================
# Map widget (simple background + axes + drone dot)
# =============================================================================
class MapWidget(QLabel):
    """
    @class MapWidget
    @brief Lightweight map view that draws a background image, axes, and the drone dot.

    @details
    Coordinates are mapped from a world square [-E, +E] to the widget rectangle,
    where +Y is up in world space and rendered as decreasing pixel V (topwards).
    """

    def __init__(self, bg_path: str, extent_half: float = 25.0, parent=None) -> None:
        """
        @brief Construct the widget and load the background image.
        @param bg_path Path to a background PNG (not rotated in code).
        @param extent_half Half-extent E (metres) for the world square.
        @param parent Qt parent widget.
        """
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setAlignment(Qt.AlignCenter)

        self._extent = float(extent_half)
        self._has_pose = False
        self._x = 0.0
        self._y = 0.0

        if bg_path and Path(bg_path).exists():
            self._bg = QPixmap(bg_path)
        else:
            self._bg = QPixmap()

    def set_pose(self, x: float, y: float) -> None:
        """
        @brief Update the current position and trigger a repaint.
        @param x World X (m)
        @param y World Y (m)
        """
        self._x, self._y = float(x), float(y)
        self._has_pose = True
        self.update()

    def _world_to_px(self, x: float, y: float, w: int, h: int) -> tuple[int, int]:
        """
        @brief Convert world coordinates to widget pixel coordinates.
        @param x World X
        @param y World Y
        @param w Widget width
        @param h Widget height
        @return (u, v) pixel coordinates
        """
        E = self._extent
        u = (x + E) / (2.0 * E) * w
        v = (E - y) / (2.0 * E) * h
        return int(u), int(v)

    def paintEvent(self, ev) -> None:
        """
        @brief Custom paint: background, frame, axes, ticks, labels, and drone dot.
        """
        p = QPainter(self)
        w, h = self.width(), self.height()

        # Background
        if not self._bg.isNull():
            p.drawPixmap(0, 0, w, h, self._bg)
        else:
            p.fillRect(0, 0, w, h, QColor(240, 240, 240))

        p.setRenderHint(QPainter.Antialiasing, True)

        # Frame
        frame_pen = QPen(QColor(40, 40, 40)); frame_pen.setWidth(2)
        p.setPen(frame_pen)
        p.drawRect(0, 0, w - 1, h - 1)

        # Axes
        axis_pen = QPen(QColor(0, 0, 0)); axis_pen.setWidth(3)
        p.setPen(axis_pen)
        p.drawLine(0, 0, 0, h - 1)          # Y axis (left)
        p.drawLine(0, h - 1, w - 1, h - 1)  # X axis (bottom)

        # Ticks every 5 m
        tick_pen = QPen(QColor(0, 0, 0)); tick_pen.setWidth(2)
        p.setPen(tick_pen)
        E = self._extent
        for xv in range(int(-E), int(E) + 1, 5):
            u = int((xv + E) / (2.0 * E) * w)
            p.drawLine(u, h - 1, u, h - 8)
        for yv in range(int(-E), int(E) + 1, 5):
            v = int((E - yv) / (2.0 * E) * h)
            p.drawLine(0, v, 7, v)

        # A couple of labels (0 and ±20) tucked inward to avoid overlap with background
        p.setFont(QFont("", 10))
        p.setPen(QColor(0, 0, 0))
        for xv in (20, 0):
            u = int((xv + E) / (2.0 * E) * w)
            display_val = -xv  # invert label sign as per your prior UI convention
            p.drawText(u - 8, h - 25, 16, 16, Qt.AlignHCenter | Qt.AlignVCenter, f"{display_val}")
        for yv in (20, 0):
            v = int((E - yv) / (2.0 * E) * h)
            display_val = -yv
            p.drawText(10, v - 8, 16, 16, Qt.AlignLeft | Qt.AlignVCenter, f"{display_val}")

        # Drone dot
        if self._has_pose:
            u, v = self._world_to_px(self._x, self._y, w, h)
            dot_pen = QPen(QColor(20, 100, 255)); dot_pen.setWidth(4)
            p.setPen(dot_pen); p.setBrush(QColor(20, 100, 255))
            p.drawEllipse(u - 3, v - 3, 6, 6)

        p.end()


# =============================================================================
# GUI (Qt) front-end
# =============================================================================
class TwoPaneGUI(QWidget):
    """
    @class TwoPaneGUI
    @brief Operator GUI with STATUS (left) and CONTROL (right) panes.

    @details
    - STATUS: status text, last command, current pose, HAG, distance/time to goal, camera preview.
    - CONTROL: goal entry, height setpoint, and a grid of action buttons.
    - Map/graph section: shows a small map with the drone dot and a CSV picker
      to generate maps/graphs via `trailblazer_utils.map_printer`.
    """

    # Qt signals receiving data from the ROS backend
    status_signal = Signal(str)
    goal_dist_signal = Signal(float)
    goal_eta_signal = Signal(float)
    pose_signal = Signal(float, float, float)
    hag_signal = Signal(float)
    camera_signal = Signal(object)  # numpy image (BGR)

    def __init__(self) -> None:
        """
        @brief Construct the GUI and connect internal signals.
        """
        super().__init__()
        self._build_ui()

        # Connect signals to UI update slots
        self.status_signal.connect(self.update_status_box)
        self.goal_dist_signal.connect(self.update_goal_distance)
        self.goal_eta_signal.connect(self.update_goal_eta)
        self.pose_signal.connect(self.update_current_position)
        self.hag_signal.connect(self.update_hag_box)
        self.camera_signal.connect(self.update_camera_view)

        # Set by main() after instantiating GuiNode
        self.ros_node: GuiNode

    # -------------------------------------------------------------------------
    # Helpers: file system / list
    # -------------------------------------------------------------------------
    def _resolve_data_dir(self) -> Path:
        """
        @brief Find the `data/` directory (cwd/data first, else repo_root/data).
        @return Path to data directory.
        """
        cwd = Path.cwd()
        data_dir = cwd / "data"
        if data_dir.exists():
            return data_dir
        repo_root = Path(__file__).resolve().parents[1]
        return repo_root / "data"

    def _refresh_csv_list(self) -> None:
        """
        @brief Populate the CSV list from the resolved data directory.
        """
        self.data_dir = self._resolve_data_dir()
        self.file_list.clear()
        if not self.data_dir.exists():
            self.file_list.addItem("No files found")
            self.file_list.setEnabled(False)
            self.btn_show_graphs.setEnabled(False)
            return
        files = sorted(self.data_dir.glob("*.csv"))
        if not files:
            self.file_list.addItem("No files found")
            self.file_list.setEnabled(False)
            self.btn_show_graphs.setEnabled(False)
            return
        self.file_list.setEnabled(True)
        for p in files:
            self.file_list.addItem(p.stem)  # show name without .csv
        self.btn_show_graphs.setEnabled(False)

    def _on_file_selected(self, curr, prev) -> None:
        """
        @brief Enable graph generation button when a real file is selected.
        """
        if curr is None or curr.text() == "No files found":
            self.btn_show_graphs.setEnabled(False)
        else:
            self.btn_show_graphs.setEnabled(True)

    # -------------------------------------------------------------------------
    # Map/graph generation
    # -------------------------------------------------------------------------
    def on_show_graphs_clicked(self) -> None:
        """
        @brief Generate path/gradient/density maps from the selected CSV and preview them.
        @details
        Uses `trailblazer_utils.map_printer.PathMapPrinter` with:
        - fixed_extent_m = 48.5
        - background: repo_root/files/blankmap.png
        Saves into repo_root/maps and previews any returned PNGs.
        """
        import traceback
        try:
            sel = self.file_list.currentItem()
            if sel is None or sel.text() == 'No files found':
                return

            csv_path = self._resolve_data_dir() / f"{sel.text()}.csv"
            if not csv_path.exists():
                raise FileNotFoundError(f"CSV not found: {csv_path}")

            repo_root = Path(__file__).resolve().parents[1]
            out_dir = repo_root / "maps"
            out_dir.mkdir(parents=True, exist_ok=True)
            bg_img = repo_root / "files" / "blankmap.png"

            from trailblazer_utils.map_printer import PathMapPrinter
            printer = PathMapPrinter(fixed_extent_m=48.5)
            result = printer.generate_from_csv(
                csv_path=csv_path,
                out_dir=out_dir,
                base_name=csv_path.stem,
                bg_img=bg_img
            )

        except Exception as e:
            tb = traceback.format_exc()
            QMessageBox.critical(self, "GENERATE GRAPHS failed", f"{e}\n\n{tb}")

    # -------------------------------------------------------------------------
    # UI construction
    # -------------------------------------------------------------------------
    def _build_ui(self) -> None:
        """
        @brief Build the complete UI, wire buttons, and populate the CSV list.
        """
        self.setWindowTitle("Trailblazer GUI")
        self.resize(QSize(1120, 300))

        # ---------- Styles ----------
        self.setStyleSheet("""
            QWidget { font-size: 14px; }
            #paneTitle { font-weight: 700; font-size: 16px; }
            #divider { background: rgba(0,0,0,.45); min-width:2px; max-width:2px; }
            #statusLabel { font-weight: 600; margin-left: 6px; }
            #lastCmdBox, #statusBoxGeneric, #goalBox, #miniBox, #videoBox, #mapBox {
                border: 2px solid #333333;
                border-radius: 4px;
                background: #ffffff;
            }
            QPushButton {
                font-size: 26px; font-weight: 700;
                padding: 14px 18px; border: none; border-radius: 18px;
            }
            QPushButton:pressed { background-color: #888888; padding-left: 16px; padding-top: 16px; }
            #btnHover    { background: #D9C40A; }
            #btnMove     { background: #64B32D; }
            #btnLand     { background: #F06A1A; }
            #btnTakeoff  { background: #0C8F24; }
            #btnEStop    { background: #CF1C12; }
        """)

        # ===== Root: STATUS | divider | CONTROL =====
        root = QHBoxLayout(self)
        root.setSpacing(0)
        root.setContentsMargins(18, 14, 18, 18)
        self.setLayout(root)

        # ----- STATUS column -----
        left = QVBoxLayout(); left.setSpacing(0.5)

        status_title = QLabel("<u>STATUS</u>")
        status_title.setObjectName("paneTitle")
        status_title.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        left.addWidget(status_title)

        # --- Status box ---
        status_container = QWidget(); st_v = QVBoxLayout(status_container)
        st_v.setContentsMargins(8, 6, 8, 6); st_v.setSpacing(1)
        st_lbl = QLabel("Status"); st_lbl.setObjectName("statusLabel")
        st_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.status_box = QFrame(); self.status_box.setObjectName("statusBoxGeneric")
        self.status_box.setFixedSize(260, 60)

        self.status_text = QLabel("CONNECTING..."); self.status_text.setAlignment(Qt.AlignCenter)
        _st_box_layout = QVBoxLayout(self.status_box)
        _st_box_layout.setContentsMargins(4, 4, 4, 4)
        _st_box_layout.addWidget(self.status_text, 0, Qt.AlignCenter)

        st_v.addWidget(st_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        st_v.addWidget(self.status_box, 0, Qt.AlignHCenter)
        left.addWidget(status_container, 0, Qt.AlignHCenter)

        # --- Last Command ---
        last_cmd_container = QWidget(); lc_v = QVBoxLayout(last_cmd_container)
        lc_v.setContentsMargins(8, 6, 8, 6); lc_v.setSpacing(1)
        lc_lbl = QLabel("Last Command"); lc_lbl.setObjectName("statusLabel")
        lc_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.last_cmd_box = QFrame(); self.last_cmd_box.setObjectName("lastCmdBox")
        self.last_cmd_box.setFixedSize(260, 60)
        self.last_cmd_text = QLabel("NO COMMAND"); self.last_cmd_text.setAlignment(Qt.AlignCenter)
        _lc_box_layout = QVBoxLayout(self.last_cmd_box)
        _lc_box_layout.setContentsMargins(4, 4, 4, 4)
        _lc_box_layout.addWidget(self.last_cmd_text, 0, Qt.AlignCenter)
        lc_v.addWidget(lc_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        lc_v.addWidget(self.last_cmd_box, 0, Qt.AlignHCenter)
        left.addWidget(last_cmd_container, 0, Qt.AlignHCenter)

        # --- Current Position ---
        pos_container = QWidget(); pos_v = QVBoxLayout(pos_container)
        pos_v.setContentsMargins(8, 6, 8, 6); pos_v.setSpacing(1)
        pos_lbl = QLabel("Current Position"); pos_lbl.setObjectName("statusLabel")
        pos_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.pos_box = QFrame(); self.pos_box.setObjectName("goalBox"); self.pos_box.setFixedSize(260, 68)
        row = QHBoxLayout(self.pos_box); row.setContentsMargins(10, 8, 10, 8); row.setSpacing(14)

        def _pair(bold_text: str) -> tuple[QWidget, QLabel]:
            name = QLabel(f"<b>{bold_text}</b>"); val = QLabel("—")
            inner = QHBoxLayout(); inner.setContentsMargins(0,0,0,0); inner.setSpacing(6)
            inner.addWidget(name, 0, Qt.AlignLeft); inner.addWidget(val, 0, Qt.AlignLeft)
            wrap = QWidget(); w = QHBoxLayout(wrap); w.setContentsMargins(0,0,0,0); w.addLayout(inner)
            return wrap, val

        x_wrap, self.pos_x_value = _pair("X:")
        y_wrap, self.pos_y_value = _pair("Y:")
        z_wrap, self.pos_z_value = _pair("Z:")
        row.addWidget(x_wrap); row.addWidget(y_wrap); row.addWidget(z_wrap)

        pos_v.addWidget(pos_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        pos_v.addWidget(self.pos_box, 0, Qt.AlignHCenter)
        left.addWidget(pos_container, 0, Qt.AlignHCenter)

        # --- HAG ---
        hag_container = QWidget(); hag_v = QVBoxLayout(hag_container)
        hag_v.setContentsMargins(8, 6, 8, 6); hag_v.setSpacing(1)
        hag_lbl = QLabel("Height Based off LIDAR"); hag_lbl.setObjectName("statusLabel")
        hag_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.hag_box = QFrame(); self.hag_box.setObjectName("miniBox"); self.hag_box.setFixedSize(260, 60)
        self.hag_text = QLabel("— m"); self.hag_text.setAlignment(Qt.AlignCenter)
        _hag_box_layout = QVBoxLayout(self.hag_box); _hag_box_layout.setContentsMargins(4,4,4,4)
        _hag_box_layout.addWidget(self.hag_text, 0, Qt.AlignCenter)
        hag_v.addWidget(hag_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        hag_v.addWidget(self.hag_box, 0, Qt.AlignHCenter)
        left.addWidget(hag_container, 0, Qt.AlignHCenter)

        # --- Distance to Goal ---
        dist_container = QWidget(); d_v = QVBoxLayout(dist_container)
        d_v.setContentsMargins(8,6,8,6); d_v.setSpacing(1)
        d_lbl = QLabel("Distance to Goal"); d_lbl.setObjectName("statusLabel")
        d_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.dist_box = QFrame(); self.dist_box.setObjectName("miniBox"); self.dist_box.setFixedSize(260, 60)
        self.dist_text = QLabel("—"); self.dist_text.setAlignment(Qt.AlignCenter)
        _d_box_layout = QVBoxLayout(self.dist_box); _d_box_layout.setContentsMargins(4,4,4,4)
        _d_box_layout.addWidget(self.dist_text, 0, Qt.AlignCenter)
        d_v.addWidget(d_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        d_v.addWidget(self.dist_box, 0, Qt.AlignHCenter)
        left.addWidget(dist_container, 0, Qt.AlignHCenter)

        # --- Time to Goal ---
        time_container = QWidget(); t_v = QVBoxLayout(time_container)
        t_v.setContentsMargins(8,6,8,6); t_v.setSpacing(1)
        t_lbl = QLabel("Time to Goal"); t_lbl.setObjectName("statusLabel")
        t_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.time_box = QFrame(); self.time_box.setObjectName("miniBox"); self.time_box.setFixedSize(260, 60)
        self.time_text = QLabel("—"); self.time_text.setAlignment(Qt.AlignCenter)
        _t_box_layout = QVBoxLayout(self.time_box); _t_box_layout.setContentsMargins(4,4,4,4)
        _t_box_layout.addWidget(self.time_text, 0, Qt.AlignCenter)
        t_v.addWidget(t_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        t_v.addWidget(self.time_box, 0, Qt.AlignHCenter)
        left.addWidget(time_container, 0, Qt.AlignHCenter)

        # --- Camera / Video ---
        video_container = QWidget(); v_v = QVBoxLayout(video_container)
        v_v.setContentsMargins(8, 6, 8, 6); v_v.setSpacing(1)
        v_lbl = QLabel("Camera"); v_lbl.setObjectName("statusLabel")
        v_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.video_box = QFrame(); self.video_box.setObjectName("videoBox"); self.video_box.setFixedSize(260, 120)
        self.video_placeholder = QLabel("No Video"); self.video_placeholder.setAlignment(Qt.AlignCenter)
        self.video_placeholder.setStyleSheet("color:#333;")
        self.video_label = QLabel(); self.video_label.setAlignment(Qt.AlignCenter); self.video_label.hide()
        _vb_layout = QVBoxLayout(self.video_box); _vb_layout.setContentsMargins(4,4,4,4)
        _vb_layout.addWidget(self.video_placeholder, 1); _vb_layout.addWidget(self.video_label, 1)
        v_v.addWidget(v_lbl, 0, Qt.AlignLeft | Qt.AlignTop); v_v.addWidget(self.video_box, 0, Qt.AlignHCenter)
        left.addWidget(video_container, 0, Qt.AlignHCenter)

        left.addStretch(1)

        # ----- Divider -----
        divider = QFrame(); divider.setObjectName("divider")
        divider.setFrameShape(QFrame.NoFrame)
        divider.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

        # ----- CONTROL column -----
        right = QVBoxLayout(); right.setSpacing(0)

        # CONTROL title
        title_row = QHBoxLayout(); title_row.setContentsMargins(0, 0, 0, 0); title_row.setSpacing(12)
        control_title = QLabel("<u>CONTROL</u>"); control_title.setObjectName("paneTitle")
        control_title.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        control_title.setStyleSheet("font-size: 16px; font-weight: 700; margin-left: 24px; min-height: 1px;")
        title_row.addWidget(control_title, 1, Qt.AlignLeft); title_row.addStretch(1)
        right.addLayout(title_row); right.addSpacing(18)

        # Container for inputs + button grid
        grid = QWidget(); grid.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        grid_col = QVBoxLayout(grid); grid_col.setSpacing(18); grid_col.setContentsMargins(24, 6, 24, 6)

        # Inputs (goal, map/graphs, height)
        inputs = QVBoxLayout(); inputs.setContentsMargins(24, 0, 24, 14); inputs.setSpacing(10)

        # Goal inputs
        goal_row_label = QLabel("Enter Goal (X, Y)"); goal_row_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        goal_row_label.setMinimumHeight(24); inputs.addWidget(goal_row_label)

        goal_row = QHBoxLayout(); goal_row.setSpacing(12)

        def _make_cell(placeholder: str) -> QLineEdit:
            le = QLineEdit(); le.setFixedSize(64, 48); le.setAlignment(Qt.AlignCenter); le.setPlaceholderText(placeholder)
            return le

        self.goal_x_edit = _make_cell("X"); self.goal_y_edit = _make_cell("Y")
        goal_row.addWidget(self.goal_x_edit); goal_row.addWidget(self.goal_y_edit)
        self.btn_set_goal = QPushButton("SET GOAL"); self.btn_set_goal.setObjectName("btnMove")
        self.btn_set_goal.setMinimumHeight(48); goal_row.addWidget(self.btn_set_goal, 1)
        inputs.addLayout(goal_row)

        # Map + Graphs block
        map_info_container = QWidget(); map_info_row = QHBoxLayout(map_info_container)
        map_info_row.setContentsMargins(8, 6, 8, 6); map_info_row.setSpacing(12)

        # Map
        map_left = QVBoxLayout()
        map_lbl = QLabel("Map"); map_lbl.setObjectName("statusLabel"); map_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.map_box = QFrame(); self.map_box.setObjectName("mapBox"); self.map_box.setFixedSize(260, 260)
        _map_layout = QVBoxLayout(self.map_box); _map_layout.setContentsMargins(4, 4, 4, 4)
        repo_root = Path(__file__).resolve().parents[1]
        bg_path = str(repo_root / "files" / "blankmap.png")
        self.map_widget = MapWidget(bg_path, extent_half=25.0)
        _map_layout.addWidget(self.map_widget, 1)
        map_left.addWidget(map_lbl, 0, Qt.AlignLeft | Qt.AlignTop); map_left.addWidget(self.map_box, 0, Qt.AlignLeft)

        # Graph generator panel
        info_panel = QVBoxLayout()
        info_title = QLabel("Graph Generation"); info_title.setObjectName("statusLabel")
        info_title.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.btn_show_graphs = QPushButton("GENERATE GRAPHS")
        self.btn_show_graphs.setObjectName("btnMove")
        self.btn_show_graphs.setMinimumHeight(48)
        self.btn_show_graphs.setEnabled(False)
        self.btn_show_graphs.clicked.connect(self.on_show_graphs_clicked)
        self.file_list = QListWidget()
        self.file_list.setSelectionMode(QListWidget.SingleSelection)
        self.file_list.currentItemChanged.connect(self._on_file_selected)
        info_panel.addWidget(info_title); info_panel.addWidget(self.btn_show_graphs); info_panel.addWidget(self.file_list, 1)

        map_info_row.addLayout(map_left, 0); map_info_row.addLayout(info_panel, 1)
        inputs.addWidget(map_info_container)

        # Height input
        height_row_label = QLabel("Set Height"); height_row_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        height_row_label.setMinimumHeight(24); inputs.addWidget(height_row_label)
        height_row = QHBoxLayout(); height_row.setSpacing(12)
        self.height_edit = QLineEdit(); self.height_edit.setMinimumHeight(48)
        self.height_edit.setAlignment(Qt.AlignCenter); self.height_edit.setPlaceholderText("Height (m)")
        height_row.addWidget(self.height_edit, 1)
        self.btn_set_height = QPushButton("SET HEIGHT"); self.btn_set_height.setObjectName("btnMove")
        self.btn_set_height.setMinimumHeight(48); height_row.addWidget(self.btn_set_height, 1)
        inputs.addLayout(height_row)

        # Final assembly
        right.addLayout(inputs)
        # Grid of big buttons
        def _cfg(b: QPushButton) -> None:
            b.setMinimumHeight(80); b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        row1 = QHBoxLayout(); row1.setSpacing(18)
        btn_hover = QPushButton("HOVER");        btn_hover.setObjectName("btnHover"); _cfg(btn_hover)
        btn_move  = QPushButton("MOVE TO GOAL"); btn_move.setObjectName("btnMove");   _cfg(btn_move)
        row1.addWidget(btn_hover, 1); row1.addWidget(btn_move, 1); grid_col.addLayout(row1)

        row2 = QHBoxLayout(); row2.setSpacing(18)
        btn_land = QPushButton("LAND");    btn_land.setObjectName("btnLand");    _cfg(btn_land)
        btn_take = QPushButton("TAKEOFF"); btn_take.setObjectName("btnTakeoff"); _cfg(btn_take)
        row2.addWidget(btn_land, 1); row2.addWidget(btn_take, 1); grid_col.addLayout(row2)

        btn_estop = QPushButton("EMERGENCY LAND"); btn_estop.setObjectName("btnEStop")
        btn_estop.setMinimumHeight(80); btn_estop.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        grid_col.addWidget(btn_estop, 0)

        right.addStretch(1)
        right.addWidget(grid, 1)

        # Root layout
        root.addLayout(left, 1); root.addWidget(divider); root.addLayout(right, 2)

        # Populate CSV list at startup
        self._refresh_csv_list()

        # ---- Wire buttons to ROS commands + last command box ----
        def _hook(btn_text: str, color_hex: str):
            return lambda: (self.ros_node.send(btn_text), self.update_last_command(btn_text, color_hex))

        btn_hover.clicked.connect(_hook("HOVER", "#D9C40A"))
        btn_move.clicked.connect(_hook("MOVE TO GOAL", "#64B32D"))
        btn_land.clicked.connect(_hook("LAND", "#F06A1A"))
        btn_take.clicked.connect(_hook("TAKEOFF", "#0C8F24"))
        btn_estop.clicked.connect(_hook("EMERGENCY LAND", "#CF1C12"))
        self.btn_set_goal.clicked.connect(self.set_goal)
        self.btn_set_height.clicked.connect(self.set_height)

    # -------------------------------------------------------------------------
    # Command/entry handlers
    # -------------------------------------------------------------------------
    def set_goal(self) -> None:
        """
        @brief Read goal X/Y from inputs and publish `/cmd/goal`.
        """
        try:
            x = float(self.goal_x_edit.text())
            y = float(self.goal_y_edit.text())
        except ValueError:
            return
        self.ros_node.publish_goal(x, y)
        self.goal_x_edit.setText(f"{x:.2f}")
        self.goal_y_edit.setText(f"{y:.2f}")

    def set_height(self) -> None:
        """
        @brief Read height from input and publish `/cmd/height`.
        """
        try:
            h = float(self.height_edit.text())
        except ValueError:
            return
        self.ros_node.publish_height(h)

    # -------------------------------------------------------------------------
    # UI updates
    # -------------------------------------------------------------------------
    def update_last_command(self, text: str, color_hex: str) -> None:
        """
        @brief Update the 'Last Command' box text and colour.
        @param text Command text.
        @param color_hex Background colour (hex).
        """
        self.last_cmd_text.setText(text)
        self.last_cmd_box.setStyleSheet(
            f"#lastCmdBox {{ border: 2px solid #333333; border-radius: 4px; background: {color_hex}; }}"
        )

    def update_status_box(self, status: str) -> None:
        """
        @brief Update status box based on incoming status string.
        @param status Human-readable status (case-insensitive mapping is applied).
        """
        status_key = status.strip().lower()
        mapping = {
            "pre flight checks": ("PRE-FLIGHT", "#CCCCCC"),
            "landed":            ("LANDED", "#F06A1A"),
            "landing":           ("LANDING", "#C75610"),
            "taking off":        ("TAKING OFF", "#0C8F24"),
            "hovering":          ("HOVERING", "#D9C40A"),
            "moving to goal":    ("MOVING TO GOAL", "#64B32D"),
            "arrived at goal":   ("ARRIVED AT GOAL", "#1A73E8"),
            "emergency landing": ("EMERGENCY LANDING", "#CF1C12"),
        }
        text, color = mapping.get(status_key, (status.upper(), "#CCCCCC"))
        self.status_text.setText(text)
        self.status_box.setStyleSheet(
            f"#statusBoxGeneric {{ border: 2px solid #333333; border-radius: 4px; background: {color}; }}"
        )

    def update_goal_position(self, x: float, y: float, z_unused: float = 0.0) -> None:
        """
        @brief Reflect a selected/active goal in the input fields (UI convenience).
        @param x Goal X
        @param y Goal Y
        @param z_unused Present for API compatibility (ignored).
        """
        self.goal_x_edit.setText(f"{x:.2f}")
        self.goal_y_edit.setText(f"{y:.2f}")

    def update_goal_distance(self, meters: float) -> None:
        """
        @brief Update the distance-to-goal readout with metric formatting.
        @param meters Distance in metres.
        """
        txt = f"{meters/1000:.2f} km" if meters >= 1000 else f"{meters:.2f} m"
        self.dist_text.setText(txt)

    def update_goal_eta(self, seconds: float) -> None:
        """
        @brief Update the time-to-goal readout (h m / m s / s).
        @param seconds Time metric in seconds (negative clears the field).
        """
        if seconds < 0:
            self.time_text.setText("—")
            return
        secs = int(seconds)
        if secs < 60:
            self.time_text.setText(f"{secs} s")
        elif secs < 3600:
            m = secs // 60; s = secs % 60
            self.time_text.setText(f"{m} m {s} s")
        else:
            h = secs // 3600; m = (secs % 3600) // 60
            self.time_text.setText(f"{h} h {m} m")

    def update_hag_box(self, hag_m: float) -> None:
        """
        @brief Update the HAG display, handling NaN cleanly.
        @param hag_m Height-above-ground (metres).
        """
        if hag_m != hag_m:  # NaN
            self.hag_text.setText("— m")
        else:
            self.hag_text.setText(f"{hag_m:.2f} m")

    def update_camera_view(self, frame) -> None:
        """
        @brief Convert a BGR numpy frame to QImage and update preview.
        @param frame Numpy image (H x W x 3, BGR).
        """
        h, w, ch = frame.shape
        qimg = QImage(frame.data, w, h, ch * w, QImage.Format_BGR888)
        pm = QPixmap.fromImage(qimg)
        self.video_label.setPixmap(pm.scaled(
            self.video_box.width()-8, self.video_box.height()-8,
            Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))
        self.video_placeholder.hide()
        self.video_label.show()

    def update_current_position(self, x: float, y: float, z: float) -> None:
        """
        @brief Update the pose readout and move the map dot.
        @param x Current X
        @param y Current Y
        @param z Current Z
        """
        self.pos_x_value.setText(f"{x:.2f}")
        self.pos_y_value.setText(f"{y:.2f}")
        self.pos_z_value.setText(f"{z:.2f}")
        if hasattr(self, "map_widget"):
            self.map_widget.set_pose(x, y)


# =============================================================================
# Entry point
# =============================================================================
def main() -> None:
    """
    @brief Start the Qt application and ROS 2 node, interleaving their event loops.
    """
    rclpy.init()
    app = QApplication(sys.argv)

    # Graceful Ctrl+C
    signal.signal(signal.SIGINT, lambda *_: QApplication.quit())

    # ROS node
    ros_node = GuiNode()

    # GUI
    gui = TwoPaneGUI()
    gui.ros_node = ros_node
    ros_node.gui_ref = gui
    gui.show()

    # Interleave ROS with Qt via timer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    timer.start(50)

    code = app.exec()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(code)


if __name__ == "__main__":
    main()
