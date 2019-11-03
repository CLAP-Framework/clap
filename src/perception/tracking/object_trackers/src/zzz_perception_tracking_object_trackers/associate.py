'''
This module provide methods to associate detections to existing trackers

Every method should have as method called associate which takes list of
trackers and detections as input. The result is a list that a list of index
to the trackers. If the detection should be initialized, then use float('nan')
'''

import numpy as np

class JointProbabilisticDataAssociationFilter:
    def __init__(self, chi_sqr_thres):
        self._chi_sqr_thres = chi_sqr_thres

    def measurent_validate(self, input):
        raise NotImplementedError()

    def associate(self, trackers, detection):
        raise NotImplementedError()

class NearestNeighborFilter:
    '''
    A naive nearest neighbor association that connect detections to closet tracker.

    XXX: Other metrics: take covariance, speed or heading angle into account
    '''
    def __init__(self, dist_thres=3, dist_metric="euclidean"):
        self._dist_thres = dist_thres
        self._metric = dist_metric

    def associate(self, detections, pose_trackers, feature_trackers):
        '''
        detections should be list of zzz_perception_msgs/DetectionBox
        pose_trackers and feature_trackers are dictionary of trackers
        '''
        if self._metric != "euclidean":
            raise ValueError("Only Euclidean distance nearest neighbor is currently supported")
        if len(detections) == 0:
            return []

        results = [float('nan')] * len(detections)
        tracker_locations = np.zeros((len(pose_trackers), 2))
        tracker_keymap = {}
        for keyidx_tr, idx_tr in enumerate(pose_trackers.keys()):
            # TODO: state tracker now ignore z axis
            tracker_locations[keyidx_tr, :] = pose_trackers[idx_tr].pose_state[:2]
            tracker_keymap[keyidx_tr] = idx_tr

        if len(pose_trackers) == 0:
            return results
        for idx_dt, dt in enumerate(detections):
            position = dt.bbox.pose.pose.position
            d = np.array([position.x, position.y]) - tracker_locations
            d = np.linalg.norm(d, axis=1)
            if np.min(d) < self._dist_thres:
                results[idx_dt] = tracker_keymap[np.argmin(d)]

        return results

class GlobalNearestNeighbor:
    '''
    Implementation of global nearest neighbor
    '''
    def __init__(self):
        raise NotImplementedError()

    def associate(self, tracker, detection):
        raise NotImplementedError()

NN = NearestNeighborFilter
JPDAF = JointProbabilisticDataAssociationFilter
