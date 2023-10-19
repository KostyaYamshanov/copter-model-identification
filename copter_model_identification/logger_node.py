import rclpy
from rclpy.node import Node

class LoggerNode(Node):

    def __init__(self) -> None:
        super().__init__("logger_node")
        self.get_logger().info("Logger node created")