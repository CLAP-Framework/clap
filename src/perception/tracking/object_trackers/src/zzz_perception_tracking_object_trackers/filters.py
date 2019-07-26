'''
This module contains filters to estimate object states.

The property of pose state should be in format
    [x, y, z, vx, vy, vz, ax, ay, az,
    [0  1  2  3   4   5   6   7   8
     rx, ry, rz, wx, wy, wz, bx, by, bz]
     9   10  11  12  13  14  15  16  17 ]
where "b" denotes angular acceleration
'''
import numpy as np
import math
import filterpy.kalman as kf

import sys
import rospy

import zzz_common.dynamic_models as dm
from zzz_perception_msgs.msg import DetectionBoxArray, DetectionBox, ObjectClass
from zzz_perception_msgs.msg import TrackingBoxArray, TrackingBox
from geometry_msgs.msg import Pose

import tf

class Box_KF:
    '''
    Kalman Filter for a box shape estimation.
    In this case, the filter is actually a least squere estimator
    '''
    def __init__(self, Q=None):
        self._filter = kf.KalmanFilter(dim_x=3, dim_z=3)
        self._filter.F = self._filter.H = np.eye(3)

        if Q is None:
            self._filter.Q = np.diag([.01, .01, .01])
        else:
            assert Q.shape == (3,3)
            self._filter.Q = Q

        self._classes = None

        self._inited = False

    def predict(self, dt):
        if not self._inited:
            return
        self._filter.predict()

    def update(self, box):
        '''
        Box should have type of zzz_perception_msgs/DetectionBox.
        None input indicates that the tracker is failed to be associated.
        '''
        assert type(box) == DetectionBox
        
        z = np.array([
            box.bbox.dimension.length_x,
            box.bbox.dimension.length_y,
            box.bbox.dimension.length_z
        ])
        R = np.array(box.bbox.dimension.covariance).reshape(3,3)
        if not self._inited:
            self._filter.x = z
            self._filter.P = R
            
            self._inited = True
        else:
            self._filter.update(z, R=R)

        # Update classification
        # XXX: Any confidence filtering?
        self._classes = box.classes

    def shape(self):
        return self._filter.x

    def shape_covariance(self):
        raise NotImplementedError()

    def classes(self):
        return self._classes

class Pose_UKF_CTRA:
    '''
    UKF using CTRA model for pose estimation

    States: [x, y, rz, v, a, w]
            [0  1  2   3  4  5]
    Observe: [x, y, rz]
    '''
    def __init__(self, Q=None):
        self._filter = kf.UnscentedKalmanFilter(dim_x=6, dim_z=3,
            dt=None, fx=dm.motion_CTRA, hx=lambda s:s[:3],
            points=kf.MerweScaledSigmaPoints(6, alpha=.1, beta=.2, kappa=-1),
        )

        if Q is None:
            self._filter.Q = np.diag([1, 1, 1, 3, 10, 3])
        else:
            assert Q.shape == (6,6)
            self._filter.Q = Q

        self._inited = False

    def predict(self, dt):
        if not self._inited:
            return
        self._filter.predict(dt=dt)

    def update(self, box):
        '''
        Box should have type of zzz_perception_msgs/DetectionBox.
        None input indicates that the tracker is failed to be associated.
        '''
        assert type(box) == DetectionBox
        
        ori = box.bbox.pose.orientation
        _,_,rz = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        R = np.array(box.bbox.pose.covariance).reshape(6, 6)
        rz = dm.wrap_angle(rz)
        if not self._inited:
            self._filter.x = np.array([
                box.bbox.pose.pose.x,
                box.bbox.pose.pose.y,
                rz,
                0,
                0,
                0,
            ])
            self._filter.P = self._filter.Q
            self._filter.P[:3,:3] = R[(0,1,5), (0,1,5)] # select x, y, rz
            
            self._inited = True
        else:
            self._filter.update(np.array([
                box.bbox.pose.pose.x,
                box.bbox.pose.pose.y,
                rz
            ]), R=R[(0,1,5), (0,1,5)])
            self._filter.x[2] = dm.wrap_angle(self._filter.x[2])
            
    @property
    def pose_state(self):
        '''
        Return states in standard form
        '''
        s = np.zeros(18)
        s[:2] = self._filter.x[:2] # x, y
        s[11] = self._filter.x[2] # rz
        s[3] = self._filter.x[3] * np.cos(self._filter.x[2]) # vx
        s[4] = self._filter.x[3] * np.sin(self._filter.x[2]) # vy
        s[6] = self._filter.x[4] * np.cos(self._filter.x[2]) # ax
        s[7] = self._filter.x[4] * np.sin(self._filter.x[2]) # ay
        s[14] = self._filter.x[5]
        raise s

    @property
    def pose_covariance(self):
        '''
        Return covariance of states in standard form

        # TODO: Return complete corresponding covariance. Hard to compute, use sampling method or directly sigma points?
        '''
        raise NotImplementedError()

class Pose_IMM:
    '''
    UKF using IMM (BR + CV + CA + CTRV + CTRA) model for pose estimation
    '''
    def __init__(self):
        raise NotImplementedError()

    def predict(self, dt):
        raise NotImplementedError()

    def update(self, z):
        raise NotImplementedError()

    def pose_state(self):
        raise NotImplementedError()

    def pose_covariance(self):
        raise NotImplementedError()
