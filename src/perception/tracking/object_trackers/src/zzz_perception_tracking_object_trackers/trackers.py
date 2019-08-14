import rospy
import numpy as np

from zzz_perception_msgs.msg import TrackingBox, TrackingBoxArray
from zzz_perception_tracking_object_trackers.filters import Pose_UKF_CV, Pose_UKF_CTRA, Box_KF
from zzz_perception_tracking_object_trackers.associate import NearestNeighborFilter

class MultiBoxTracker:
    def __init__(self, lost_count=6, # TODO: add track_count to filter false positives
        pose_tracker_cls=Pose_UKF_CV, feature_tracker_cls=Box_KF,
        pose_tracker_args=None, feature_tracker_args=None,
        associate_filter=NearestNeighborFilter()):
        self._tracked_objects = dict() # Object trackers
        self._tracked_features = dict() # Feature trackers (shape, class, etc)
        self._counter_track = dict() # Count for frames tracked consecutively
        self._counter_lost = dict() # Count for frames lost consecutively

        self._last_timestamp = None
        self._last_frameid = None
        self._counter_id = 0 # Counter for tracked objects
        self._lost_count = lost_count

        self._pose_tracker_cls = pose_tracker_cls
        self._feature_trackers_cls = feature_tracker_cls
        self._associator = associate_filter

    def _initialize(self, detection):
        '''
        Initialize a new target
        '''
        pose_tracker = self._pose_tracker_cls()
        feature_tracker = self._feature_trackers_cls()

        pose_tracker.update(detection)
        feature_tracker.update(detection)

        self._tracked_objects[self._counter_id] = pose_tracker
        self._tracked_features[self._counter_id] = feature_tracker
        self._counter_track[self._counter_id] = 1
        self._counter_lost[self._counter_id] = 0
        self._counter_id += 1

    def update(self, array):
        '''
        Detections here should be of type zzz_perception_msgs/DetectionBoxArray
        '''
        rospy.logdebug("Update with new detection array (count: %d)", len(array.detections))
        if self._last_timestamp is None:
            # Initialize all trackers
            for dtarget in array.detections:
                self._initialize(dtarget)
        else:
            # Match existing trackers
            dt = array.header.stamp.to_sec() - self._last_timestamp.to_sec()
            for tracker in self._tracked_objects.values():
                tracker.predict(dt)
            for tracker in self._tracked_features.values():
                tracker.predict(dt)

            association = self._associator.associate(detections=array.detections,
                pose_trackers=self._tracked_objects,
                feature_trackers=self._tracked_features
            )
            tr_idlist = set(self._tracked_objects.keys())
            for idx_dt, idx_tr in enumerate(association):
                if idx_tr is None:
                    continue # invalid detection
                if np.isnan(idx_tr):
                    # Initialize this detector
                    self._initialize(array.detections[idx_dt])
                else:
                    self._tracked_objects[idx_tr].update(array.detections[idx_dt])
                    self._tracked_features[idx_tr].update(array.detections[idx_dt])
                    self._counter_lost[idx_tr] = 0
                    self._counter_track[idx_tr] += 1
                    if idx_tr in tr_idlist:
                        tr_idlist.remove(idx_tr)

            for idx_tr in tr_idlist:
                self._counter_lost[idx_tr] += 1
                self._counter_track[idx_tr] = 0
                
            
        # Deal with out-dated or invalid trackers
        rm_list = []
        for idx_tr in self._tracked_objects:
            if self._counter_lost[idx_tr] > self._lost_count:
            # TODO: if self._tracked_objects[idx_tr].covariance.determinant > 100:
                rm_list.append(idx_tr)
        for idx_tr in rm_list:
            del self._tracked_objects[idx_tr]

        # Update times
        self._last_timestamp = array.header.stamp
        self._last_frameid = array.header.frame_id

    def report(self):
        '''
        Return the message for valid tracked objects
        '''
        array = TrackingBoxArray()
        array.header.stamp = self._last_timestamp
        array.header.frame_id = self._last_frameid
        for idx_tr in self._tracked_objects.keys():
            trackbox = TrackingBox()
            trackbox.classes = self._tracked_features[idx_tr].classes
            trackbox.bbox.pose.pose.position.x = self._tracked_objects[idx_tr].pose_state[0]
            trackbox.bbox.pose.pose.position.y = self._tracked_objects[idx_tr].pose_state[1]
            trackbox.bbox.pose.pose.position.z = self._tracked_objects[idx_tr].pose_state[2]
            trackbox.bbox.dimension.length_x = self._tracked_features[idx_tr].shape_state[0]
            trackbox.bbox.dimension.length_y = self._tracked_features[idx_tr].shape_state[1]
            trackbox.bbox.dimension.length_z = self._tracked_features[idx_tr].shape_state[2]
            trackbox.twist.twist.linear.x = self._tracked_objects[idx_tr].pose_state[3]
            trackbox.twist.twist.linear.y = self._tracked_objects[idx_tr].pose_state[4]
            trackbox.twist.twist.linear.z = self._tracked_objects[idx_tr].pose_state[5]
            # TODO: Filling out more fields
            trackbox.uid = idx_tr
            # TODO: Add option to track these targets using static coordinate
            # FIXME: This is a very naive confidence report
            trackbox.confidence = min(1, (self._counter_track[idx_tr] + 5) / 10.)
            array.targets.append(trackbox)

        rospy.logdebug("Reported tracked objects (count: %d)", len(array.targets))
        return array
