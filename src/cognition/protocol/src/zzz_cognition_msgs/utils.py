from zzz_common.kinematics import get_absolute_state
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_cognition_msgs.msg import LaneState, MapState, RoadObstacle, JunctionMapState
from zzz_perception_msgs.msg import TrackingBoxArray, ObjectClass, DimensionWithCovariance

import numpy as np
import tf

def default_msg(msg_type):
    '''
    Setting default values for the messages
    '''
    if msg_type == LaneState:
        msg = LaneState()
        msg.stop_distance = float('inf')
    elif msg_type == MapState:
        msg = MapState()
    elif msg_type == RoadObstacle:
        msg = RoadObstacle()
    elif msg_type == JunctionMapState:
        msg = JunctionMapState()
    else:
        raise ValueError("Unrecognized message type")
    
    return msg

def convert_tracking_box(array, pose):
    '''
    Convert tracking box into RoadObstacle. Pose should be RigidBodyStateStamped which is got from /zzz/navigation/ego_pose
    '''
    assert type(array) == TrackingBoxArray
    assert type(pose) == RigidBodyStateStamped

    obstacles = []
    
    for obj in array.targets:
        if array.header.frame_id != pose.header.frame_id:
            trackpose = RigidBodyStateStamped()
            trackpose.header = array.header
            trackpose.state.pose = obj.bbox.pose
            trackpose.state.twist = obj.twist
            trackpose.state.accel = obj.accel
            abspose = get_absolute_state(trackpose, pose)

            assert abspose.header.frame_id == 'map'
        else:
            abspose = RigidBodyStateStamped()
            abspose.header = array.header
            abspose.state.pose = obj.bbox.pose
            abspose.state.twist = obj.twist
            abspose.state.accel = obj.accel

        obstacle = RoadObstacle()
        obstacle.uid = obj.uid
        obstacle.state = abspose.state
        if len(obj.classes) > 0:
            obstacle.cls = obj.classes[0]
        else:
            obstacle.cls.classid = ObjectClass.UNKNOWN
            obstacle.cls.score = 1

        # Convert obstacle shape
        obstacle.shape_type = RoadObstacle.SHAPE_BOX
        obstacle.sbox = obj.bbox.dimension

        # TODO(zyxin): These lines correct faults in rotation?
        ori = obj.bbox.pose.pose.orientation
        rotation_mat = tf.transformations.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])[:3, :3]
        rotation_mat_inverse = np.linalg.inv(rotation_mat)

        vel_self = np.array([[obj.twist.twist.linear.x], [obj.twist.twist.linear.y], [obj.twist.twist.linear.z]])
        vel_world = np.matmul(rotation_mat_inverse, vel_self)
        obstacle.state.twist.twist.linear.x = vel_world[0]
        obstacle.state.twist.twist.linear.y = vel_world[1]
        obstacle.state.twist.twist.linear.z = vel_world[2]
        obstacles.append(obstacle)

    return obstacles
