import rclpy
from mavros_msgs.msg import RCIn, RCOut
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import *

class LoggerNode(Node):

    def __init__(self) -> None:
        super().__init__("logger_node")
        self.get_logger().info("Logger node created")

        # https://answers.ros.org/question/360676/qos-python-code-for-create_subscriber/
        odom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback, odom_qos_profile)
        self.rc_in_sub = self.create_subscription(
            RCIn,
            "/mavros/rc/in",
            self.rc_in_callback, 10)
        self.rc_out_sub = self.create_subscription(
            RCOut,
            "/mavros/rc/out",
            self.rc_out_callback, 10)
        # prevent unused variable warning
        # self.odom_sub
        self.rc_in_sub
        self.rc_out_sub

    def odom_callback(self, odom_msg):
        self.get_logger().info("odom_callback")
        self.get_logger().info("{}".format(odom_msg.header.stamp.nanosec))
    
    def rc_in_callback(self, rc_in_msg):
        self.get_logger().info("rc_in_callback")
        self.get_logger().info("{}".format(rc_in_msg.header.stamp.nanosec))
        self.get_logger().info("{}".format(rc_in_msg.channels[0:4]))
    
    def rc_out_callback(self, rc_out_msg):
        self.get_logger().info("rc_out_callback")
        self.get_logger().info("{}".format(rc_out_msg.header.stamp.nanosec))
        self.get_logger().info("{}".format(rc_out_msg.channels[0:4]))