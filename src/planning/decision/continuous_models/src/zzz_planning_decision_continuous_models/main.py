#!/usr/bin/env python

from zzz_cognition_msgs.msg import MapState


class MainDecision(object):
    def __init__(self, trajectory_planner=None):
        self._dynamic_map_buffer = None
        self._trajectory_planner = trajectory_planner

    def receive_dynamic_map(self, dynamic_map):
        assert type(dynamic_map) == MapState
        self._dynamic_map_buffer = dynamic_map

    def update_trajectory(self, close_to_junction=40):

        if self._dynamic_map_buffer is None:
            return None
        dynamic_map = self._dynamic_map_buffer

        if dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP and dynamic_map.mmap.distance_to_junction > close_to_junction:
            self._trajectory_planner.clear_buff(dynamic_map)
            return None
        elif dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP:
            msg = self._trajectory_planner.build_frenet_path(dynamic_map)
            return None
        else:
            return self._trajectory_planner.trajectory_update(dynamic_map)

        


    

    
