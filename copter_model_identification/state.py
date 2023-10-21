from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from rclpy.clock import Clock
from scipy.spatial.transform import Rotation
import numpy as np

class Timer:
    def __init__(self) -> None:
        self.conversion_const = 10 ** 9
        self.initial_time = 0
        self.current_time = 0
        
    def update_initial(self) -> None:
        self.initial_time = Clock().now().nanoseconds / self.conversion_const

    def update_current(self) -> None:
        self.current_time = Clock().now().nanoseconds / self.conversion_const

    def get_time(self) -> float:
        self.update_current()
        return self.current_time - self.initial_time


class RC:
    def __init__(self) -> None:
        self.x = 0.
        self.y = 0
        self.z = 0
        self.r = 0
    
    def from_rc_msg(self, rc) -> None:
        self.x = rc.channels[0]
        self.y = rc.channels[1]
        self.z = rc.channels[2]
        self.r = rc.channels[3]

    def table_head(self, prefix = "") -> str:
        if prefix == "":
            return "R1 R2 R3 R4 "
        
        return "R1 R2 R3 R4 ".replace(" ", "_{} ".format(prefix))

    def to_string(self) -> str:
        return "{:.0f} {:.0f} {:.0f} {:.0f}".format(self.x, self.y, self.z, self.r)

class DroneState():
    def __init__(self) -> None:
        self.orientation = Vector3()
        self.position = Vector3()
        self.linear_twist = Vector3()
        self.angular_twist = Vector3()
    
    def position_from_odom(self, odom : Odometry) ->Vector3:
        position = Vector3()
        position.x = odom.pose.pose.position.x
        position.y = odom.pose.pose.position.y
        position.z = odom.pose.pose.position.z
        return position
    
    def rpy_from_odom(self, odom : Odometry) -> Vector3:
        orient = Rotation.from_quat([
            np.float(odom.pose.pose.orientation.x),
            np.float(odom.pose.pose.orientation.y),
            np.float(odom.pose.pose.orientation.z),
            np.float(odom.pose.pose.orientation.w)]
        ).as_euler('xyz')
        
        rpy = Vector3()
        rpy.x = orient[0]
        rpy.y = orient[1]
        rpy.z = orient[2]
        return rpy

    def from_odom(self, odom) -> None:
        self.position = self.position_from_odom(odom)
        self.orientation = self.rpy_from_odom(odom)
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular

    def table_head(self) -> str:
        return "Roll Pitch Yaw X Y Z Vx Vy Vz Wx Wy Wz "

    def to_string(self) -> str:
        rpy_str = "{:.3f} {:.3f} {:.3f} ".format(self.orientation.x, self.orientation.y, self.orientation.z)
        pos_str = "{:.3f} {:.3f} {:.3f} ".format(self.position.x, self.position.y, self.position.z)
        lin_twist_str = "{:.3f} {:.3f} {:.3f} ".format(self.linear_twist.x, self.linear_twist.y, self.linear_twist.z)
        ang_twist_str = "{:.3f} {:.3f} {:.3f}".format(self.angular_twist.x, self.angular_twist.y, self.angular_twist.z)
        
        return rpy_str + pos_str + lin_twist_str + ang_twist_str