from rclpy.node import Node
from rclpy.qos import *

from mavros_msgs.msg import RCIn, RCOut
from copter_model_identification.state import State, RC
from nav_msgs.msg import Odometry

class LoggerNode(Node):

    def __init__(self) -> None:
        super().__init__("logger_node")
        self.get_logger().info("Logger node created")

        self.curr_state = State() # current copter state
        self.curr_rc_in = RC()  # control from RC
        self.curr_rc_out = RC() # control from motors

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
        self.odom_sub
        self.rc_in_sub
        self.rc_out_sub

    def fill_table(self):
        pass
    
    def odom_callback(self, odom_msg) -> None:
        self.get_logger().info("odom_callback")
        # self.get_logger().info("{}".format(odom_msg.header.stamp.nanosec))
        self.curr_state.from_odom(odom_msg)
        self.get_logger().info(self.curr_state.to_string())

    def rc_in_callback(self, rc_in_msg) -> None:
        self.get_logger().info("rc_in_callback")
        # self.get_logger().info("{}".format(rc_in_msg.header.stamp.nanosec))
        # self.get_logger().info("{}".format(rc_in_msg.channels[0:4]))
    
    def rc_out_callback(self, rc_out_msg) -> None:
        self.get_logger().info("rc_out_callback")
        # self.get_logger().info("{}".format(rc_out_msg.header.stamp.nanosec))
        # self.get_logger().info("{}".format(rc_out_msg.channels[0:4]))
    