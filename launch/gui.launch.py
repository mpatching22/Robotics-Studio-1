import sys
import rclpy
from rclpy.node import Node
from PySide6.QtWidgets import QApplication, QWidget
from PySide6.QtCore import QSize


class GuiNode(Node):
    """
    A simple ROS2 node that opens an empty GUI window.
    """
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info("GUI Node started.")


def main(args=None):
    rclpy.init(args=args)

    # Qt Application
    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("Trailblazer GUI")
    window.resize(QSize(500, 400))  # Set to 500x400 px
    window.show()

    # Start ROS2 node
    gui_node = GuiNode()

    # Run Qt event loop (blocks until window closed)
    app.exec()

    # Shutdown ROS2 cleanly
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
