import rospy
import numpy as np
import pcl
import tf
import tf2_ros as tf2
from sensor_msgs.msg import PointCloud2

class ExtrinsicFilter:
    '''
    TODO: Also accept object detection result as input
    '''
    def __init__(self, **params):
        self._params = params
        self._tfbuffer = tf2.Buffer()
        self._tflistener = tf2.TransformListener(self._tfbuffer)

    def filter(self, msg):
        assert type(msg) == PointCloud2

        cloud = pcl.PointCloud(msg)
        try:
            rt = self._tfbuffer.lookup_transform(msg.header.frame_id, self._params['target_frame'], msg.header.stamp)
        except tf2.LookupException as e:
            rospy.logwarn("TF lookup error: %s", str(e))
            return None

        euler = tf.transformations.quaternion_matrix([rt.transform.rotation.x, rt.transform.rotation.y, rt.transform.rotation.z, rt.transform.rotation.w])
        homo = cloud.xyz - [rt.transform.translation.x, rt.transform.translation.y, rt.transform.translation.z]
        cloud.xyz[:,:] = homo.dot(euler[:3,:3])

        nmsg = cloud.to_msg()
        nmsg.header.stamp = msg.header.stamp
        nmsg.header.frame_id = self._params['target_frame']
        return nmsg

class PointCloudBufferFilter:
    '''
    TODO: Implement multiple point cloud aggregator using icp or other method
    '''
