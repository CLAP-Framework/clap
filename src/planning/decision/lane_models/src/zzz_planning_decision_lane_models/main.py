#!/usr/bin/env python

import rospy
import numpy as np
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed, get_yaw
from zzz_planning_decision_lane_models.local_trajectory import PolylineTrajectory # TODO(Temps): Should seperate into continous models

# Make lat lon model as parameter

class MainDecision(object):
    def __init__(self, lon_decision=None, lat_decision=None, local_trajectory=None):
        self._dynamic_map_buffer = None
        self._static_map_buffer = None

        self._longitudinal_model_instance = lon_decision
        self._lateral_model_instance = lat_decision
        self._local_trajectory_instance = PolylineTrajectory() # MPCTrajectory()
        self._local_trajectory_instance_for_ref = PolylineTrajectory() # TODO(Temps): Should seperate into continous models

    # receive_dynamic_map running in subscriber callBack thread.
    def receive_dynamic_map(self, dynamic_map):
        self._dynamic_map_buffer = dynamic_map

    # update running in main node thread loop
    def update(self):
        '''
        This function generate trajectory
        '''
        # update_dynamic_local_map
        if self._dynamic_map_buffer is None:
            return None

        trajectory = None
        dmap = self._dynamic_map_buffer
        changing_lane_index, desired_speed = self._lateral_model_instance.lateral_decision(dmap)
        if desired_speed < 0: # TODO: clean this
            desired_speed = 0
        rospy.logdebug("target_lane_index = %d, target_speed = %f km/h", changing_lane_index, desired_speed*3.6)
        
        # TODO(Temps): Should seperate into continous models 
        if changing_lane_index == -1:
            trajectory = self._local_trajectory_instance_for_ref.get_trajectory(dmap, changing_lane_index, desired_speed)#FIXME(ksj)
        else:
            trajectory = self._local_trajectory_instance.get_trajectory(dmap, changing_lane_index, desired_speed)


        msg = DecisionTrajectory()
        msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory) # TODO: move to library
        msg.desired_speed = desired_speed

        return msg

    def convert_ndarray_to_pathmsg(self, path): 
        msg = Path()
        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            msg.poses.append(pose)
        msg.header.frame_id = "map"

        return msg

    
