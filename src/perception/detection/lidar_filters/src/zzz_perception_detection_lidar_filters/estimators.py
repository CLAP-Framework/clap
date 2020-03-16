
import tf
import pcl
import numpy as np
from zzz_perception_msgs.msg import DetectionBoxArray
from geometry_msgs.msg import Quaternion
import sensor_msgs.point_cloud2 as pc2

np.seterr(all='raise')

class LShapeFilter:
    def __init__(self, **params):
        self._params = params

    def filter(self, array):
        assert type(array) == DetectionBoxArray

        for target in array.detections:
            self._fit_shape(target)
        return array

    def _fit_shape(self, target):
        cloud = pcl.PointCloud(target.source_cloud)
        xyz = cloud.xyz

        pmax = np.max(xyz, axis=0)
        pmin = np.min(xyz, axis=0)
        target.bbox.pose.pose.position.x = (pmax[0] + pmin[0])/2
        target.bbox.pose.pose.position.y = (pmax[1] + pmin[1])/2
        target.bbox.pose.pose.position.z = (pmax[2] + pmin[2])/2

        angle = xyz[:,1] / xyz[:,0]
        imax, imin = np.argmax(angle), np.argmin(angle)
        assert imax != imin

        x1, y1, _ = xyz[imin, :]
        x2, y2, _ = xyz[imax, :]
        dist = np.abs((y2-y1)*xyz[:,0] - (x2-x1)*xyz[:,1] + x2*y1 - x1*y2) / np.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))
        iend = np.argmax(dist)
        
        x0, y0, _ = xyz[iend, :]
        dist2min = np.sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0))
        dist2max = np.sqrt((x2-x0)*(x2-x0) + (y2-y0)*(y2-y0))
        if dist2min == 0 and dist2max == 0:
            rospy.logwarn("[LShape filter] Invalid shape!")

        if dist2max > dist2min:
            yaw = np.arctan2(y2-y0, x2-x0)
            target.bbox.dimension.length_x = dist2max
            target.bbox.dimension.length_y = dist2min
        else:
            yaw = np.arctan2(y1-y0, x1-x0)
            target.bbox.dimension.length_x = dist2min
            target.bbox.dimension.length_y = dist2max
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        target.bbox.pose.pose.orientation = Quaternion(*q)

        # TODO: Add reasonable covariances
        target.bbox.pose.covariance = np.diag([1] * 6).flatten().tolist()
        target.bbox.dimension.covariance = np.diag([1] * 3).flatten().tolist()

        return target
