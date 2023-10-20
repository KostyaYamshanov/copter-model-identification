from rclpy.node import Node
from rclpy.qos import *
from mavros_msgs.msg import RCIn, RCOut
from copter_model_identification.state import State, RC
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
# import tf2_ros


class LoggerNode(Node):

    def __init__(self) -> None:
        super().__init__("logger_node")
        self.get_logger().info("Logger node created")

        self.curr_state = State() # current copter state
        self.curr_rc_in = RC()    # control from RC
        self.curr_rc_out = RC()   # control from motors
        self.dt = 0.02            # sec

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

        self.timer = self.create_timer(self.dt, self.save_data)

        # prevent unused variable warning
        self.odom_sub
        self.rc_in_sub
        self.rc_out_sub

    def save_data(self):
        self.get_logger().info("save data")
        self.get_logger().info("{} {} {}".format(self.curr_state.to_string(), self.curr_rc_in.to_string(), self.curr_rc_out.to_string()))

    def odom_callback(self, odom_msg) -> None:
        self.curr_state.from_odom(odom_msg)

    def rc_in_callback(self, rc_in_msg) -> None:
        self.get_logger().info("rc_in_callback")
        self.curr_rc_in.from_rc_msg(rc_in_msg)
    
    def rc_out_callback(self, rc_out_msg) -> None:
        self.get_logger().info("rc_out_callback")
        self.curr_rc_out.from_rc_msg(rc_out_msg)
    