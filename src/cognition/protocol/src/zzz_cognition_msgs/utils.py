from zzz_cognition_msgs.msg import LaneState, MapState, RoadObstacle

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
