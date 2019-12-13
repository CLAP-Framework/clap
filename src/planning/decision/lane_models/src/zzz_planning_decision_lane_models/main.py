#!/usr/bin/env python

import rospy
import numpy as np
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from zzz_planning_msgs.msg import DecisionTrajectory
from threading import Lock
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed, get_yaw

from zzz_planning_decision_lane_models.get_trajectory_by_polyline import PolylineTrajectory #FIXME(ksj)
from zzz_planning_decision_lane_models.get_trajectory_by_MPC import MPCTrajectory #FIXME(ksj)

# Make lat lon model as parameter

class MainDecision(object):
    def __init__(self, lon_decision=None, lat_decision=None):
        self._dynamic_map_buffer = None
        self._static_map_buffer = None

        self._longitudinal_model_instance = lon_decision
        self._lateral_model_instance = lat_decision
        self._dynamic_map_lock = Lock()
        self._get_trajectory_by_polyline_instance = PolylineTrajectory() #FIXME(ksj)
        self._get_trajectory_by_mpc_instance = MPCTrajectory()

    # receive_dynamic_map running in Subscriber CallBack Thread.
    def receive_dynamic_map(self, dynamic_map):
        with self._dynamic_map_lock:
            self._dynamic_map_buffer = dynamic_map
    def receive_static_map(self,static_map):
        self._static_map_buffer = static_map
    # update running in main node thread loop
    def update(self):
        '''
        This function generate trajectory
        '''
        if self._static_map_buffer is None:
            return None
        # update_dynamic_local_map
        if self._dynamic_map_buffer is None:
            return None

        trajectory = None
        with self._dynamic_map_lock:
            changing_lane_index, desired_speed = self._lateral_model_instance.lateral_decision(self._dynamic_map_buffer)
            if desired_speed < 0: # TODO: clean this
                desired_speed = 0

            rospy.logdebug("target_lane_index = %d, target_speed = %f km/h", changing_lane_index, desired_speed*3.6)
            # TODO: Is this reasonable?
            # if len(self.dynamic_map.jmap.reference_path.map_lane.central_path_points) == 0:
            #     return DecisionTrajectory() # Return null trajectory
            # get trajectory by target lane and desired speed
            if changing_lane_index==-1:
                trajectory = self._get_trajectory_by_polyline_instance.get_trajectory(self._dynamic_map_buffer, changing_lane_index, desired_speed)#FIXME(ksj)
            else:
                trajectory = self._get_trajectory_by_mpc_instance.get_trajectory(self._dynamic_map_buffer, changing_lane_index, desired_speed)


        msg = DecisionTrajectory()
        msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory) # TODO: move to library
        msg.desired_speed = desired_speed  #FIXME(NANSHAN)

        return msg

    def convert_ndarray_to_pathmsg(self, path):#FIXME(dns )
        msg = Path()
        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            msg.poses.append(pose)
        msg.header.frame_id = "map"  # FIXME(NANSHAN):

        return msg

    
