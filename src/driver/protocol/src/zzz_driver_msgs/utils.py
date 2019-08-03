import math
import tf.transformations as tft

from zzz_driver_msgs.msg import RigidBodyState
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance

def get_speed(msg):
    '''
    Calculate speed value
    '''
    if type(msg) == Twist:
        return math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
    elif type(msg) == TwistWithCovariance or type(msg) == RigidBodyState:
        return get_speed(msg.twist)
    else:
        raise ValueError("Incorrect message type for get_speed")

def get_yaw(msg):
    '''
    Calculate yaw angle assuming on 2d plane
    '''
    if type(msg) == Pose:
        _,_,yaw = tft.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        return yaw
    elif type(msg) == PoseWithCovariance or type(msg) == RigidBodyState:
        return get_yaw(msg.pose)
    else:
        raise ValueError("Incorrect message type for get_yaw")
