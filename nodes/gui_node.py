#!/usr/bin/env python3
# Trailblazer GUI — two-pane layout with full-width row buttons (no ROS).

import sys, signal
from PySide6.QtCore import Qt, QSize, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QHBoxLayout, QVBoxLayout,
    QFrame, QPushButton, QSizePolicy
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String   # or whatever message type you need

from PySide6.QtGui import QImage, QPixmap

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from cv_bridge import CvBridge
import cv2

from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLineEdit

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')   # ✔ give the node a name

        # Single publisher
        self.pub_cmd = self.create_publisher(String, '/cmd/control', 10)
        
        # Subscriber to movement status
        self.sub_status = self.create_subscription(
            String, '/movement/status', self.status_callback, 10
        )

        # --- New: goal/position (x,y,z), goal/distance, goal/time, camera ---
        self.bridge = CvBridge()

         # NEW: publishers for goal + height
        self.pub_goal   = self.create_publisher(PointStamped, '/cmd/goal', 10)
        self.pub_height = self.create_publisher(Float32, '/cmd/height', 10)

        # NEW: pose + camera subscribers - taken out due to lag
        # self.bridge = CvBridge()
        # self.sub_cam = self.create_subscription(Image, '/camera/image', self.camera_cb, 10) 

        self.sub_pose_ps = self.create_subscription(
            PoseStamped, '/drone/pose_1hz', self.pose_callback_ps, 10
        )
        
        self.sub_hag = self.create_subscription(
            Float32, '/altitude/hag', self.hag_cb, 10
        )
                
        self.sub_goal_dist = self.create_subscription(
            Float32, '/goal/distance', self.goal_dist_cb, 10
        )
        self.sub_goal_time = self.create_subscription(
            Float32, '/goal/time', self.goal_time_cb, 10
        )


    def status_callback(self, msg: String):
        status_text = msg.data
        if hasattr(self, 'gui_ref') and self.gui_ref:
            self.gui_ref.status_signal.emit(status_text)   # emit Qt signal

    def goal_dist_cb(self, msg: Float32):
        if hasattr(self, 'gui_ref') and self.gui_ref:
            self.gui_ref.goal_dist_signal.emit(float(msg.data))

    def goal_time_cb(self, msg: Float32):
        if hasattr(self, 'gui_ref') and self.gui_ref:
            # seconds as float; GUI formats to mm:ss
            self.gui_ref.goal_eta_signal.emit(float(msg.data))
    def hag_cb(self, msg: Float32):
        if hasattr(self, 'gui_ref') and self.gui_ref:
            # allow NaN to show as "—"
            self.gui_ref.hag_signal.emit(float(msg.data))

    def camera_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if hasattr(self, 'gui_ref') and self.gui_ref:
                self.gui_ref.camera_signal.emit(frame)
        except Exception as e:
            self.get_logger().warn(f'Camera conversion failed: {e}')

    def send(self, msg_text: str):
        msg = String()
        msg.data = msg_text
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"Published: {msg_text}")

    
    def pose_callback_ps(self, msg: PoseStamped):
        if hasattr(self, 'gui_ref') and self.gui_ref:
            p = msg.pose.position
            self.gui_ref.pose_signal.emit(float(p.x), float(p.y), float(p.z))

    def pose_callback_pcs(self, msg: PoseWithCovarianceStamped):
        if hasattr(self, 'gui_ref') and self.gui_ref:
            p = msg.pose.pose.position
            self.gui_ref.pose_signal.emit(float(p.x), float(p.y), float(p.z))

    def camera_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if hasattr(self, 'gui_ref') and self.gui_ref:
                self.gui_ref.camera_signal.emit(frame)
        except Exception as e:
            self.get_logger().warn(f'Camera conversion failed: {e}')

    # convenience methods to publish from GUI
    def publish_goal(self, x: float, y: float):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y, msg.point.z = x, y, 0.0
        self.pub_goal.publish(msg)
        self.get_logger().info(f"Published /cmd/goal: ({x}, {y})")


    def publish_height(self, h: float):
        self.pub_height.publish(Float32(data=float(h)))
        self.get_logger().info(f"Published /cmd/height: {h}")
    
class TwoPaneGUI(QWidget):
    status_signal = Signal(str)   # <--- Qt signal carrying status text
    goal_dist_signal = Signal(float)
    goal_eta_signal  = Signal(float)
    camera_signal    = Signal(object)  # numpy/cv2 frame
    pose_signal   = Signal(float, float, float)  # X, Y, Z
    hag_signal = Signal(float)
    camera_signal = Signal(object)               # cv2 frame (numpy array)


    def __init__(self):
        super().__init__()
        self._build_ui()
        self.status_signal.connect(self.update_status_box)
        self.goal_dist_signal.connect(self.update_goal_distance)
        self.goal_eta_signal.connect(self.update_goal_eta)
        self.camera_signal.connect(self.update_camera_view)
        self.status_signal.connect(self.update_status_box)
        self.pose_signal.connect(self.update_current_position)
        self.hag_signal.connect(self.update_hag_box)
        self.camera_signal.connect(self.update_camera_view)


    def _build_ui(self):
        self.setWindowTitle("Trailblazer GUI")
        self.resize(QSize(1120, 300))
        self.status_signal.connect(self.update_status_box)

        # ---------- Styles ----------
        self.setStyleSheet("""
            QWidget { font-size: 14px; }
            #paneTitle { font-weight: 700; font-size: 16px; }

            #divider { background: rgba(0,0,0,.45); min-width:2px; max-width:2px; }

            /* STATUS boxes */
            #statusLabel { font-weight: 600; margin-left: 6px; }
            #lastCmdBox, #statusBoxGeneric {
                border: 2px solid #333333;
                border-radius: 4px;
                background: #ffffff;        /* change lastCmdBox bg later when wiring logic */
            }
                           
            QPushButton {
                font-size: 26px; font-weight: 700;
                padding: 14px 18px;
                border: none;
                border-radius: 18px;
            }
            QPushButton:pressed {
                background-color: #888888;
                padding-left: 16px;
                padding-top: 16px;
            }

            #btnHover    { background: #D9C40A; }
            #btnMove    { background: #64B32D; }
            #btnLand    { background: #F06A1A; }
            #btnTakeoff { background: #0C8F24; }
            #btnEStop   { background: #CF1C12; }
                           
            #goalBox, #miniBox, #videoBox {
                border: 2px solid #333333;
                border-radius: 4px;
                background: #ffffff;
            }
                           
            #goalBox {
                border: 2px solid #333333;
                border-radius: 4px;
                background: #ffffff;
            }
            #mapBox {
                border: 2px solid #333333;
                border-radius: 4px;
                background: #ffffff;
            }
        """)

        # ===== Root: STATUS | divider | CONTROL =====
        root = QHBoxLayout(self)
        root.setSpacing(0)                        # flush to the divider
        root.setContentsMargins(18, 14, 18, 18)   # outer padding
        self.setLayout(root)

        # ----- STATUS column -----
        left = QVBoxLayout()
        left.setSpacing(0.5)

        status_title = QLabel("<u>STATUS</u>")
        status_title.setObjectName("paneTitle")
        status_title.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        left.addWidget(status_title)

        # --- Status box ---
        status_container = QWidget()
        st_v = QVBoxLayout(status_container)
        st_v.setContentsMargins(8, 6, 8, 6)
        st_v.setSpacing(1)

        st_lbl = QLabel("Status")
        st_lbl.setObjectName("statusLabel")
        st_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.status_box = QFrame()
        self.status_box.setObjectName("statusBoxGeneric")
        self.status_box.setFixedSize(260, 60)

        self.status_text = QLabel("CONNECTING...")
        self.status_text.setAlignment(Qt.AlignCenter)
        _st_box_layout = QVBoxLayout(self.status_box)
        _st_box_layout.setContentsMargins(4, 4, 4, 4)
        _st_box_layout.addWidget(self.status_text, 0, Qt.AlignCenter)

        st_v.addWidget(st_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        st_v.addWidget(self.status_box, 0, Qt.AlignHCenter)
        left.addWidget(status_container, 0, Qt.AlignHCenter)
        
        # --- Last Command box (label top-left, box centered) ---
        last_cmd_container = QWidget()
        lc_v = QVBoxLayout(last_cmd_container)
        lc_v.setContentsMargins(8, 6, 8, 6)
        lc_v.setSpacing(1)

        lc_lbl = QLabel("Last Command")
        lc_lbl.setObjectName("statusLabel")
        lc_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.last_cmd_box = QFrame()
        self.last_cmd_box.setObjectName("lastCmdBox")
        self.last_cmd_box.setFixedSize(260, 60)

        # Text inside the box
        self.last_cmd_text = QLabel("NO COMMAND")
        self.last_cmd_text.setAlignment(Qt.AlignCenter)
        # put the text inside the frame with a tiny layout
        _lc_box_layout = QVBoxLayout(self.last_cmd_box)
        _lc_box_layout.setContentsMargins(4, 4, 4, 4)
        _lc_box_layout.addWidget(self.last_cmd_text, 0, Qt.AlignCenter)

        lc_v.addWidget(lc_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        lc_v.addWidget(self.last_cmd_box, 0, Qt.AlignHCenter)
        left.addWidget(last_cmd_container, 0, Qt.AlignHCenter)

        # --- Current Position (X/Y/Z from /pose) ---
        pos_container = QWidget()
        pos_v = QVBoxLayout(pos_container)
        pos_v.setContentsMargins(8, 6, 8, 6)
        pos_v.setSpacing(1)

        pos_lbl = QLabel("Current Position")
        pos_lbl.setObjectName("statusLabel")
        pos_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.pos_box = QFrame()
        self.pos_box.setObjectName("goalBox")   # uses same style as other boxes
        self.pos_box.setFixedSize(260, 68)

        row = QHBoxLayout(self.pos_box)
        row.setContentsMargins(10, 8, 10, 8)
        row.setSpacing(14)

        def pair(bold_text: str):
            name = QLabel(f"<b>{bold_text}</b>")
            val  = QLabel("—")
            inner = QHBoxLayout(); inner.setContentsMargins(0,0,0,0); inner.setSpacing(6)
            inner.addWidget(name, 0, Qt.AlignLeft)
            inner.addWidget(val, 0, Qt.AlignLeft)
            wrap = QWidget(); w = QHBoxLayout(wrap); w.setContentsMargins(0,0,0,0); w.addLayout(inner)
            return wrap, val

        x_wrap, self.pos_x_value = pair("X:")
        y_wrap, self.pos_y_value = pair("Y:")
        z_wrap, self.pos_z_value = pair("Z:")

        row.addWidget(x_wrap)
        row.addWidget(y_wrap)
        row.addWidget(z_wrap)

        pos_v.addWidget(pos_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        pos_v.addWidget(self.pos_box, 0, Qt.AlignHCenter)
        left.addWidget(pos_container, 0, Qt.AlignHCenter)

        # --- Height Based off LIDAR ---
        hag_container = QWidget()
        hag_v = QVBoxLayout(hag_container)
        hag_v.setContentsMargins(8, 6, 8, 6)
        hag_v.setSpacing(1)

        hag_lbl = QLabel("Height Based off LIDAR")
        hag_lbl.setObjectName("statusLabel")
        hag_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.hag_box = QFrame()
        self.hag_box.setObjectName("miniBox")
        self.hag_box.setFixedSize(260, 60)

        self.hag_text = QLabel("— m")
        self.hag_text.setAlignment(Qt.AlignCenter)
        _hag_box_layout = QVBoxLayout(self.hag_box)
        _hag_box_layout.setContentsMargins(4,4,4,4)
        _hag_box_layout.addWidget(self.hag_text, 0, Qt.AlignCenter)

        hag_v.addWidget(hag_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        hag_v.addWidget(self.hag_box, 0, Qt.AlignHCenter)
        left.addWidget(hag_container, 0, Qt.AlignHCenter)


        # --- Distance to Goal ---
        dist_container = QWidget()
        d_v = QVBoxLayout(dist_container)
        d_v.setContentsMargins(8,6,8,6)
        d_v.setSpacing(1)

        d_lbl = QLabel("Distance to Goal")
        d_lbl.setObjectName("statusLabel")
        d_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.dist_box = QFrame()
        self.dist_box.setObjectName("miniBox")
        self.dist_box.setFixedSize(260, 60)

        self.dist_text = QLabel("—")
        self.dist_text.setAlignment(Qt.AlignCenter)
        _d_box_layout = QVBoxLayout(self.dist_box)
        _d_box_layout.setContentsMargins(4,4,4,4)
        _d_box_layout.addWidget(self.dist_text, 0, Qt.AlignCenter)

        d_v.addWidget(d_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        d_v.addWidget(self.dist_box, 0, Qt.AlignHCenter)
        left.addWidget(dist_container, 0, Qt.AlignHCenter)

        # --- ~Time to Goal ---
        time_container = QWidget()
        t_v = QVBoxLayout(time_container)
        t_v.setContentsMargins(8,6,8,6)
        t_v.setSpacing(1)

        t_lbl = QLabel("Time to Goal")
        t_lbl.setObjectName("statusLabel")
        t_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.time_box = QFrame()
        self.time_box.setObjectName("miniBox")
        self.time_box.setFixedSize(260, 60)

        self.time_text = QLabel("—")
        self.time_text.setAlignment(Qt.AlignCenter)
        _t_box_layout = QVBoxLayout(self.time_box)
        _t_box_layout.setContentsMargins(4,4,4,4)
        _t_box_layout.addWidget(self.time_text, 0, Qt.AlignCenter)

        t_v.addWidget(t_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        t_v.addWidget(self.time_box, 0, Qt.AlignHCenter)
        left.addWidget(time_container, 0, Qt.AlignHCenter)

        # --- Camera / Video box ---
        video_container = QWidget()
        v_v = QVBoxLayout(video_container)
        v_v.setContentsMargins(8, 6, 8, 6)
        v_v.setSpacing(1)

        v_lbl = QLabel("Camera")
        v_lbl.setObjectName("statusLabel")
        v_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.video_box = QFrame()
        self.video_box.setObjectName("videoBox")
        self.video_box.setFixedSize(260, 120)

        self.video_placeholder = QLabel("No Video")
        self.video_placeholder.setAlignment(Qt.AlignCenter)
        self.video_placeholder.setStyleSheet("color:#333;")

        self.video_label = QLabel()           # where the pixmap will go
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.hide()               # hidden until first frame arrives

        _vb_layout = QVBoxLayout(self.video_box)
        _vb_layout.setContentsMargins(4,4,4,4)
        _vb_layout.addWidget(self.video_placeholder, 1)
        _vb_layout.addWidget(self.video_label, 1)

        v_v.addWidget(v_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        v_v.addWidget(self.video_box, 0, Qt.AlignHCenter)
        left.addWidget(video_container, 0, Qt.AlignHCenter)

        left.addStretch(1)
        
        # ----- Divider (1 px) -----
        divider = QFrame()
        divider.setObjectName("divider")
        divider.setFrameShape(QFrame.NoFrame)
        divider.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

        # ----- CONTROL column -----
        right = QVBoxLayout()
        right.setSpacing(0)

        control_title = QLabel("<u>CONTROL</u>")
        control_title.setObjectName("paneTitle")
        control_title.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        right.addWidget(control_title)
        right.addSpacing(18)  # small gap under title

        # Container that holds the rows of buttons.
        grid = QWidget()
        # IMPORTANT: let this widget expand horizontally, but stay compact vertically.
        grid.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        grid_col = QVBoxLayout(grid)
        grid_col.setSpacing(18)
        # Inner padding so buttons don't hug the column edges.
        grid_col.setContentsMargins(24, 6, 24, 6)

        # --- Reusable button config ---
        def config_btn(b: QPushButton):
            b.setMinimumHeight(80)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # ======= CONTROL INPUTS (above the big buttons) =======
        inputs = QVBoxLayout()
        inputs.setContentsMargins(24, 0, 24, 14)
        inputs.setSpacing(10)

        # --- Enter Goal: X Y Z + SET GOAL ---
        goal_row_label = QLabel("Enter Goal (X, Y)")
        goal_row_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        goal_row_label.setMinimumHeight(24)
        inputs.addWidget(goal_row_label)

        goal_row = QHBoxLayout()
        goal_row.setSpacing(12)

        def make_cell(placeholder):
            le = QLineEdit()
            le.setFixedSize(64, 48)
            le.setAlignment(Qt.AlignCenter)
            le.setPlaceholderText(placeholder)
            return le

        self.goal_x_edit = make_cell("X")
        self.goal_y_edit = make_cell("Y")

        goal_row.addWidget(self.goal_x_edit)
        goal_row.addWidget(self.goal_y_edit)

        self.btn_set_goal = QPushButton("SET GOAL")
        self.btn_set_goal.setObjectName("btnMove")
        self.btn_set_goal.setMinimumHeight(48)
        goal_row.addWidget(self.btn_set_goal, 1)

        inputs.addLayout(goal_row)

        # --- Map (placeholder) ---
        map_container = QWidget()
        map_v = QVBoxLayout(map_container)
        map_v.setContentsMargins(8, 6, 8, 6)
        map_v.setSpacing(4)

        map_lbl = QLabel("Map")
        map_lbl.setObjectName("statusLabel")
        map_lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.map_box = QFrame()
        self.map_box.setObjectName("mapBox")
        self.map_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.map_box.setFixedHeight(160)   # tweak height as you like

        # Inner layout with "No Map"
        self.map_placeholder = QLabel("No Map")
        self.map_placeholder.setAlignment(Qt.AlignCenter)

        _map_layout = QVBoxLayout(self.map_box)
        _map_layout.setContentsMargins(4, 4, 4, 4)
        _map_layout.addWidget(self.map_placeholder, 1)

        map_v.addWidget(map_lbl, 0, Qt.AlignLeft | Qt.AlignTop)
        map_v.addWidget(self.map_box, 0)

        # Add into the inputs stack between Enter Goal and Set Height
        inputs.addWidget(map_container)


        # --- Set Height + SET HEIGHT ---
        height_row_label = QLabel("Set Height")
        height_row_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        height_row_label.setMinimumHeight(24)
        inputs.addWidget(height_row_label)

        height_row = QHBoxLayout()
        height_row.setSpacing(12)

        self.height_edit = QLineEdit()
        self.height_edit.setMinimumHeight(48)
        self.height_edit.setAlignment(Qt.AlignCenter)
        self.height_edit.setPlaceholderText("Height (m)")
        height_row.addWidget(self.height_edit, 1)

        self.btn_set_height = QPushButton("SET HEIGHT")
        self.btn_set_height.setObjectName("btnMove")   # green style
        self.btn_set_height.setMinimumHeight(48)
        height_row.addWidget(self.btn_set_height, 1)

        inputs.addLayout(height_row)

        # Put the inputs above the grid of big buttons
        right.addLayout(inputs)

        # Row 1: HOVER | MOVE TO GOAL
        row1 = QHBoxLayout(); row1.setSpacing(18)
        btn_hover = QPushButton("HOVER");        btn_hover.setObjectName("btnHover"); config_btn(btn_hover)
        btn_move  = QPushButton("MOVE TO GOAL"); btn_move.setObjectName("btnMove");   config_btn(btn_move)
        # Equal widths across the row:
        row1.addWidget(btn_hover, 1)
        row1.addWidget(btn_move, 1)
        grid_col.addLayout(row1)


        # Row 2: LAND | TAKEOFF
        row2 = QHBoxLayout(); row2.setSpacing(18)
        btn_land = QPushButton("LAND");     btn_land.setObjectName("btnLand");       config_btn(btn_land)
        btn_take = QPushButton("TAKEOFF");  btn_take.setObjectName("btnTakeoff");    config_btn(btn_take)
        row2.addWidget(btn_land, 1)
        row2.addWidget(btn_take, 1)
        grid_col.addLayout(row2)

        # Row 3: EMERGENCY LAND (full width)
        btn_estop = QPushButton("EMERGENCY LAND"); btn_estop.setObjectName("btnEStop")
        btn_estop.setMinimumHeight(80)
        btn_estop.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        grid_col.addWidget(btn_estop, 0)  # full width within grid

        # Keep the grid compact (no vertical stretching); centered by side margins.
        right.addStretch(1)  # push content upward a little, like the mockup
        right.addWidget(grid, 1)

        # Assemble root with 1:2 width ratio (STATUS : CONTROL)
        root.addLayout(left, 1)
        root.addWidget(divider)
        root.addLayout(right, 2)

    # (Optional) expose buttons later if you want to connect signals
    # via properties or by storing them as self.btn_*.

        def _hook(btn_text, color_hex):
            return lambda: (self.ros_node.send(btn_text), self.update_last_command(btn_text, color_hex))
        
        btn_hover.clicked.connect(_hook("HOVER", "#D9C40A"))
        btn_move.clicked.connect(_hook("MOVE TO GOAL", "#64B32D"))
        btn_land.clicked.connect(_hook("LAND", "#F06A1A"))
        btn_take.clicked.connect(_hook("TAKEOFF", "#0C8F24"))
        btn_estop.clicked.connect(_hook("EMERGENCY LAND", "#CF1C12"))

        self.btn_set_goal.clicked.connect(self.set_goal)
        self.btn_set_height.clicked.connect(self.set_height)

    def set_goal(self):
        try:
            x = float(self.goal_x_edit.text())
            y = float(self.goal_y_edit.text())
        except ValueError:
            return
        self.ros_node.publish_goal(x, y)
        self.goal_x_edit.setText(f"{x:.2f}")
        self.goal_y_edit.setText(f"{y:.2f}")


    def set_height(self):
        try:
            h = float(self.height_edit.text())
        except ValueError:
            return
        self.ros_node.publish_height(h)


    def update_last_command(self, text: str, color_hex: str):
        # Set text
        self.last_cmd_text.setText(text)
        # Color the box background (keep border from stylesheet)
        self.last_cmd_box.setStyleSheet(
            f"#lastCmdBox {{ border: 2px solid #333333; border-radius: 4px; background: {color_hex}; }}"
        )

    def update_status_box(self, status: str):
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

    
    def update_goal_position(self, x: float, y: float, z_unused: float = 0.0):
        self.goal_x_edit.setText(f"{x:.2f}")
        self.goal_y_edit.setText(f"{y:.2f}")



    def update_goal_distance(self, meters: float):
        txt = f"{meters/1000:.2f} km" if meters >= 1000 else f"{meters:.2f} m"
        self.dist_text.setText(txt)

    def update_goal_eta(self, seconds: float):
        if seconds < 0:
            self.time_text.setText("—")
            return

        secs = int(seconds)

        if secs < 60:
            # Less than a minute
            self.time_text.setText(f"{secs} s")
        elif secs < 3600:
            # Less than an hour
            m = secs // 60
            s = secs % 60
            self.time_text.setText(f"{m} m {s} s")
        else:
            # An hour or more
            h = secs // 3600
            m = (secs % 3600) // 60
            self.time_text.setText(f"{h} h {m} m")

    def update_hag_box(self, hag_m: float):
        if hag_m != hag_m:  # NaN check
            self.hag_text.setText("— m")
        else:
            self.hag_text.setText(f"{hag_m:.2f} m")

    def update_camera_view(self, frame):
        # frame is a cv2 BGR image (H x W x 3)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_BGR888)
        pm = QPixmap.fromImage(qimg)
        self.video_label.setPixmap(pm.scaled(
            self.video_box.width()-8, self.video_box.height()-8,
            Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))
        self.video_placeholder.hide()
        self.video_label.show()

    def update_current_position(self, x: float, y: float, z: float):
        self.pos_x_value.setText(f"{x:.2f}")
        self.pos_y_value.setText(f"{y:.2f}")
        self.pos_z_value.setText(f"{z:.2f}")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: QApplication.quit())
    
    # ROS node
    ros_node = GuiNode()

    # GUI
    w = TwoPaneGUI()
    w.ros_node = ros_node   # pass node into GUI
    ros_node.gui_ref = w   # let the node call GUI updates
    w.show()

    # Allow Qt to process ROS2 events
    from PySide6.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(50)

    sys.exit(app.exec())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    