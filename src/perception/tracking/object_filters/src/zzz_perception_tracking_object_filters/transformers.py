import rospy
import numpy as np
import tf
import tf.transformations as tft
from zzz_perception_msgs.msg import DetectionBoxArray

class RigidToStaticTransformer:
    '''
    This module transform detection/tracking result into a static frame (i.e. odom/map) using RigidBodyState.
    '''
    def __init__(self, **params):
        self._params = params
        self._ego_pose = None

    def receive_pose(self, msg):
        self._ego_pose = msg

    def filter(self, msg):
        if self._ego_pose == None:
            raise RuntimeError("Transformer only works when poses are received!")

        q = [self._ego_pose.state.pose.pose.orientation.x, self._ego_pose.state.pose.pose.orientation.y,
             self._ego_pose.state.pose.pose.orientation.z, self._ego_pose.state.pose.pose.orientation.w]
        R = tf.transformations.quaternion_matrix(q)[:3,:3]
        T = [self._ego_pose.state.pose.pose.position.x, self._ego_pose.state.pose.pose.position.y, self._ego_pose.state.pose.pose.position.z]

        if type(msg) == DetectionBoxArray:
            for detection in msg.detections:
                old_pos = [detection.bbox.pose.pose.position.x, detection.bbox.pose.pose.position.y, detection.bbox.pose.pose.position.z]
                new_pos = np.array(old_pos).dot(R.T) + T
                old_ori = [detection.bbox.pose.pose.orientation.x, detection.bbox.pose.pose.orientation.y,
                           detection.bbox.pose.pose.orientation.z, detection.bbox.pose.pose.orientation.w]
                new_ori = tft.quaternion_multiply(q, old_ori)

                detection.bbox.pose.pose.position.x = new_pos[0]
                detection.bbox.pose.pose.position.y = new_pos[1]
                detection.bbox.pose.pose.position.z = new_pos[2]
                detection.bbox.pose.pose.orientation.x = new_ori[0]
                detection.bbox.pose.pose.orientation.y = new_ori[1]
                detection.bbox.pose.pose.orientation.z = new_ori[2]
                detection.bbox.pose.pose.orientation.w = new_ori[3]
                # TODO: convert covariance

            msg.header.frame_id = self._ego_pose.header.frame_id
            return msg
        else:
            raise TypeError("Not supported type!")

