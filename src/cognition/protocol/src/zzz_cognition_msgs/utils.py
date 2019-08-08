from zzz_cognition_msgs.msg import LaneState, MapState, RoadObstacle, JunctionMapState
from zzz_perception_msgs.msg import TrackingBox

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

def convert_tracking_box(obj):
    '''
    Convert tracking box into RoadObstacle
    '''
    assert type(obj) == TrackingBox
    
    obstacle = RoadObstacle()
    obstacle.uid = obj.uid
    obstacle.state.pose = obj.bbox.pose
    obstacle.state.twist = obj.twist
    obstacle.state.accel = obj.accel
    obstacle.cls = obj.classes[0]
    # TODO: Convert obstacle shape

    return obstacle
