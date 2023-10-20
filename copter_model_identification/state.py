from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class Timer:
    def __init__(self) -> None:
        pass

class RC:
    def __init__(self) -> None:
        self.x = 0.
        self.y = 0
        self.z = 0
        self.r = 0
    
    def to_string(self) -> str:
        return "{} {} {} {}".format(self.x, self.y, self.z, self.r)

class State():
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
        # TODO
        return Vector3()

    def from_odom(self, odom) -> None:
        self.position = self.position_from_odom(odom)
        self.orientation = self.rpy_from_odom(odom)
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular

    def table_head(self) -> str:
        return "Roll Pitch Yaw X Y Z Vx Vy Vz Wx Wy Wz"

    def to_string(self) -> str:
        rpy_str = "{} {} {} ".format(self.orientation.x, self.orientation.y, self.orientation.z)
        pos_str = "{} {} {} ".format(self.position.x, self.position.y, self.position.z)
        lin_twist_str = "{} {} {} ".format(self.linear_twist.x, self.linear_twist.y, self.linear_twist.z)
        ang_twist_str = "{} {} {}".format(self.angular_twist.x, self.angular_twist.y, self.angular_twist.z)
        
        return rpy_str + pos_str + lin_twist_str + ang_twist_str