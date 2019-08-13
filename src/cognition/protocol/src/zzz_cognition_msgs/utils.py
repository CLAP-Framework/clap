from zzz_common.kinematics import get_absolute_state
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_cognition_msgs.msg import LaneState, MapState, RoadObstacle, JunctionMapState
from zzz_perception_msgs.msg import TrackingBoxArray, ObjectClass

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
    
    return msg

def convert_tracking_box(array, pose):
    '''
    Convert tracking box into RoadObstacle. Pose should be RigidBodyStateStamped which is got from /zzz/navigation/ego_pose
    '''
    assert type(array) == TrackingBoxArray
    assert type(pose) == RigidBodyStateStamped

    obstacles = []
    
    for obj in array.targets:
        trackpose = RigidBodyStateStamped()
        trackpose.header = array.header
        trackpose.state.pose = obj.bbox.pose
        trackpose.state.twist = obj.twist
        trackpose.state.accel = obj.accel
        abspose = get_absolute_state(trackpose, pose)

        assert abspose.header.frame_id == 'map'

        obstacle = RoadObstacle()
        obstacle.uid = obj.uid
        obstacle.state = abspose.state
        if len(obj.classes) > 0:
            obstacle.cls = obj.classes[0]
        else:
            obstacle.cls.classid = ObjectClass.UNKNOWN
            obstacle.cls.score = 1
        # TODO: Convert obstacle shape

        obstacles.append(obstacle)

    return obstacles
