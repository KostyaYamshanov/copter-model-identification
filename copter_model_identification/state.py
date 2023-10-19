import math
from geometry_msgs.msg import Pose, Quaternion, Vector3


class RC:

    def __init__(self) -> None:
        self.x = 0.
        self.y = 0
        self.z = 0
        self.r = 0
        self.buttons = 0

class State:
    def __init__(self) -> None:
        self.position = Pose()
        self.orientation = Quaternion()
        self.linear_twist = Vector3()
        self.angular_twist = Vector3()