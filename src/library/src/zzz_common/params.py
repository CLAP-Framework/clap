'''
This module provides consistent params loading from rosparam and command line input
'''

import argparse
import rospy
from addict import Dict as edict

try:
    # ROS related items
    import rospy
    from sensor_msgs.msg import CameraInfo

    use_ros = True
except ImportError:
    use_ros = False

# TODO: create a fake ArgumentParser to show additional information and support complex arguments
def parse_private_args(**kvargs):
    '''
    This function provides the ability to get params directly from ros param service.
    '''

    parser = argparse.ArgumentParser()
    node_name = rospy.get_name()

    ros_args = {}

    if use_ros:
        for name, default in kvargs.items():
            parser.add_argument('--' + name, default=default, type=type(default))
            ros_args[name] = rospy.get_param(node_name + '/' + name, default)
    
    else:
        # Use builtin arg parser when run the function without ROS
        cmd_args = parser.parse_args()
        for k, v in vars(cmd_args).items():
            if k in ros_args and v != ros_args[v]:
                rospy.logwarn("Duplicate arguments {}: {} / {}".format(k, ros_args[k], v))
            if v != kvargs[k]:
                ros_args[k] = v
    return edict(ros_args)

class IntrinsicListener:
    '''
    This class is for easier access to sensor intrinsics. The parameters of sensors are correlated with its frame_id.
    Currently the class only support camera intrinsics, XXX in future it may support various kinds of sensor intrinsic
        by a unified params representation (e.g. json, msgpack, proto).
    '''
    def __init__(self):
        if not use_ros:
            raise NotImplementedError("The CameraInfoListener class can only be used under ROS!")

        self._buffer = dict()
        self._subscriber = rospy.Subscriber("/intri_static", CameraInfo, self._caminfo_callback)

    def _caminfo_callback(self, msg):
        if sum(msg.K) == 0:
            rospy.logwarn("The intrinsics of the camera is not correctly set! Please check your camera manager!")

        self._buffer[msg.header.frame_id] = msg

    def lookupCameraInfo(self, cam_frame, time=None, timeout=1):
        '''
        TODO: Currently only latest message is preserved
        '''
        if cam_frame not in self._buffer:
            return None
        return self._buffer[cam_frame]
    
class StaticCameraInfoBroadcaster:
    def __init__(self):
        if not use_ros:
            raise NotImplementedError("The StaticCameraInfoBroadcaster class can only be used under ROS!")

        self._publisher = rospy.Publisher("/intri_static", CameraInfo, latch=True, queue_size=1)

    def sendCameraInfo(self, msg):
        self._publisher.publish(msg)
