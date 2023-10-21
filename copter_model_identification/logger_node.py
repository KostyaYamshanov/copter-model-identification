from rclpy.node import Node
from rclpy.qos import *
import rclpy
from mavros_msgs.msg import RCIn, RCOut, State
from copter_model_identification.state import DroneState, RC, Timer
from nav_msgs.msg import Odometry
# import tf2_ros


class LoggerNode(Node):

    def __init__(self) -> None:
        super().__init__("logger_node")
        self.get_logger().info("Logger node created")

        self.curr_state = DroneState() # current copter state
        self.curr_rc_in = RC()    # control from RC
        self.curr_rc_out = RC()   # control from motors
        self.dt = 0.1             # sec
        self.is_armed = False
        self.timer = Timer()
        test_path = "/home/kostya/PHD/code/phd_ws/src/copter-model-identification/copter_model_identification/data.csv"
        self.data = open(test_path, 'w')

        odom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            depth=1)
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
        self.drone_state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback, 10)

        self.save_data_timer = self.create_timer(self.dt, self.save_data)

        header = "{}{}{}{}\n".format("T ", self.curr_rc_in.table_head("in"), self.curr_state.table_head(), self.curr_rc_out.table_head("out"))
        self.data.write(header)

        # prevent unused variable warning
        self.odom_sub
        self.rc_in_sub
        self.rc_out_sub
        self.save_data_timer

        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def save_data(self) -> None:
        if (self.is_armed):
            time = self.timer.get_time()
            line = "{:.1f} {} {} {}\n".format(time, self.curr_rc_in.to_string(), self.curr_state.to_string(), self.curr_rc_out.to_string())
            self.data.write(line)

    def odom_callback(self, odom_msg) -> None:
        self.curr_state.from_odom(odom_msg)

    def rc_in_callback(self, rc_in_msg) -> None:
        self.curr_rc_in.from_rc_msg(rc_in_msg)
    
    def rc_out_callback(self, rc_out_msg) -> None:
        self.curr_rc_out.from_rc_msg(rc_out_msg)
    
    def state_callback(self, state_msg) -> None:
        # First Arm
        if ((not self.is_armed) and state_msg.armed):
            self.timer.update_initial()
            self.timer.update_current()
            self.is_armed = state_msg.armed
        
        #  Disarm
        if (self.is_armed and (not state_msg.armed)):
            self.is_armed = state_msg.armed
            raise SystemExit

    def on_shutdown(self):
        self.get_logger().warn("Logger node shutdown!")
        self.data.close()