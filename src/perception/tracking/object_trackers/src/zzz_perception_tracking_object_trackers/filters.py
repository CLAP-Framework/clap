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
import numpy.linalg as npl
import math
import filterpy.kalman as kf

import sys
import rospy

import zzz_common.dynamic_models as dm
from zzz_common.geometry import wrap_angle
from zzz_perception_msgs.msg import DetectionBoxArray, DetectionBox, ObjectClass
from zzz_perception_msgs.msg import TrackingBoxArray, TrackingBox
from geometry_msgs.msg import Pose

import tf

def isPD(B):
    """Returns true when input is positive-definite, via Cholesky"""
    try:
        _ = npl.cholesky(B)
        return True
    except npl.LinAlgError:
        return False

def nearestPD(A):
    """Find the nearest positive-definite matrix to input
    A Python/Numpy port of John D'Errico's `nearestSPD` MATLAB code [1], which
    credits [2].
    [1] https://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd
    [2] N.J. Higham, "Computing a nearest symmetric positive semidefinite
    matrix" (1988): https://doi.org/10.1016/0024-3795(88)90223-6
    """

    B = (A + A.T) / 2
    _, s, V = npl.svd(B)

    H = np.dot(V.T, np.dot(np.diag(s), V))

    A2 = (B + H) / 2

    A3 = (A2 + A2.T) / 2

    if isPD(A3):
        return A3

    spacing = np.spacing(npl.norm(A))
    # The above is different from [1]. It appears that MATLAB's `chol` Cholesky
    # decomposition will accept matrixes with exactly 0-eigenvalue, whereas
    # Numpy's will not. So where [1] uses `eps(mineig)` (where `eps` is Matlab
    # for `np.spacing`), we use the above definition. CAVEAT: our `spacing`
    # will be much larger than [1]'s `eps(mineig)`, since `mineig` is usually on
    # the order of 1e-16, and `eps(1e-16)` is on the order of 1e-34, whereas
    # `spacing` will, for Gaussian random matrixes of small dimension, be on
    # othe order of 1e-16. In practice, both ways converge, as the unit test
    # below suggests.
    I = np.eye(A.shape[0])
    k = 1
    while not isPD(A3):
        mineig = np.min(np.real(npl.eigvals(A3)))
        A3 += I * (-mineig * k**2 + spacing)
        k += 1

    return A3

def _check_covariance(filter_, err_thres=1):
    if not isPD(filter_.P):
        nP = nearestPD(filter_.P)
        pdiff = np.abs(nP - filter_.P)
        filter_.P = nP

        if np.sum(pdiff) > err_thres: # Raise error when the difference is too large
            rospy.logerr("Covariance matrix is far from positive-definite")
            rospy.logdebug("Matrix difference:\n" + str(pdiff))
            return False
        else:
            rospy.logwarn("Non positive-definite P matrix encountered! (total diff: %f)", np.sum(pdiff))

    return True

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
        ], dtype='f8')
        R = np.array(box.bbox.dimension.covariance, dtype='f8').reshape(3,3)
        assert isPD(R), "Covariance matrix should be positive definite"
        if not self._inited:
            self._filter.x = z
            self._filter.P = R
            
            self._inited = True
        else:
            self._filter.update(z, R=R)

        # Update classification
        # XXX: Any confidence filtering?
        self._classes = box.classes

    @property
    def shape_state(self):
        return self._filter.x

    @property
    def shape_covariance(self):
        raise self._filter.P

    @property
    def classes(self):
        return self._classes

class Pose_UKF_CV:
    '''
    UKF using CV model for pose estimation

    States: [x, y, vx, vy]
            [0  1  2   3 ]
    Observe: [x, y]
             [0  1]
    '''
    def __init__(self, Q=None):
        self._filter = kf.UnscentedKalmanFilter(dim_x=4, dim_z=2,
            dt=None, fx=dm.motion_CV, hx=lambda s:s[:2],
            points=kf.MerweScaledSigmaPoints(4, alpha=.1, beta=.2, kappa=-1)
        )

        if Q is None:
            self._filter.Q = np.diag([1., 1., 5., 5.])
        else:
            assert Q.shape == (4,4)
            self._filter.Q = Q

        self._inited = False

    def predict(self, dt):
        if not self._inited:
            return
        # TODO: Remove covariance exploded tracker
        _check_covariance(self._filter)
        self._filter.predict(dt=dt)

    def update(self, box):
        '''
        Box should have type of zzz_perception_msgs/DetectionBox.
        None input indicates that the tracker is failed to be associated.
        '''
        assert type(box) == DetectionBox
        
        R = np.array(box.bbox.pose.covariance, dtype='f8').reshape(6, 6)
        assert isPD(R), "Covariance matrix should be positive definite"
        if not self._inited:
            self._filter.x = np.array([
                box.bbox.pose.pose.position.x,
                box.bbox.pose.pose.position.y,
                0,
                0,
            ], dtype='f8')
            self._filter.P = self._filter.Q
            self._filter.P[:2,:2] = R[:2,:2]
            
            self._inited = True
        else:
            self._filter.update(np.array([
                box.bbox.pose.pose.position.x,
                box.bbox.pose.pose.position.y,
            ], dtype='f8'), R=R[:2,:2])
            
    @property
    def pose_state(self):
        '''
        Return states in standard form
        '''
        s = np.zeros(18)
        s[:2] = self._filter.x[:2] # x, y
        s[3:5] = self._filter.x[2:] # vx
        return s

    @property
    def pose_covariance(self):
        '''
        Return covariance of states in standard form
        '''
        sigma = np.diag([np.inf] * 18)
        sigma[np.ix_([1,2,4,5], [1,2,4,5])] = self._filter.P
        return sigma

class Pose_UKF_CTRA:
    '''
    UKF using CTRA model for pose estimation

    States: [x, y, rz, v, a, w]
            [0  1  2   3  4  5]
    Observe: [x, y, rz]
             [0  1  2 ]
    '''
    def __init__(self, Q=None):
        self._filter = kf.UnscentedKalmanFilter(dim_x=6, dim_z=3,
            dt=None, fx=dm.motion_CTRA, hx=lambda s:s[:3],
            points=kf.MerweScaledSigmaPoints(6, alpha=.1, beta=.2, kappa=-1),
            x_mean_fn=self._state_mean, z_mean_fn=self._state_mean,
            residual_x=self._state_diff, residual_z=self._state_diff,
        )

        if Q is None:
            self._filter.Q = np.diag([1., 1., 1., 3., 10., 3.])
        else:
            assert Q.shape == (6,6)
            self._filter.Q = Q

        self._inited = False

    def _state_mean(self, sigmas, Wm):
        '''
        XXX: Any way to use complex number or quaternion-like value to prevent this?
             A discussion here: https://math.stackexchange.com/questions/2621677/extended-kalman-filter-equation-for-orientation-quaternion
        '''
        x = np.average(sigmas, axis=0, weights=Wm)
        s = np.average(np.sin(sigmas[:,2]), axis=0, weights=Wm)
        c = np.average(np.cos(sigmas[:,2]), axis=0, weights=Wm)
        x[2] = np.arctan2(s, c)
        return x

    def _state_diff(self, x, y):
        d = x - y
        d[2] = wrap_angle(d[2])
        return d

    def predict(self, dt):
        if not self._inited:
            return
        
        # TODO: Covariance explode when predict
        _check_covariance(self._filter)
        self._filter.predict(dt=dt)

    def update(self, box):
        '''
        Box should have type of zzz_perception_msgs/DetectionBox.
        None input indicates that the tracker is failed to be associated.
        '''
        assert type(box) == DetectionBox
        
        ori = box.bbox.pose.pose.orientation
        _,_,rz = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        R = np.array(box.bbox.pose.covariance, dtype='f8').reshape(6, 6)
        assert isPD(R), "Covariance matrix should be positive definite"
        rz = wrap_angle(rz)
        if not self._inited:
            self._filter.x = np.array([
                box.bbox.pose.pose.position.x,
                box.bbox.pose.pose.position.y,
                rz,
                0,
                0,
                0,
            ], dtype='f8')
            self._filter.P = self._filter.Q
            self._filter.P[:3,:3] = R[np.ix_([0,1,5], [0,1,5])] # select x, y, rz
            
            self._inited = True
        else:
            self._filter.update(np.array([
                box.bbox.pose.pose.position.x,
                box.bbox.pose.pose.position.y,
                rz
            ], dtype='f8'), R=R[np.ix_([0,1,5], [0,1,5])])
            self._filter.x[2] = wrap_angle(self._filter.x[2])
            
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
        return s

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
