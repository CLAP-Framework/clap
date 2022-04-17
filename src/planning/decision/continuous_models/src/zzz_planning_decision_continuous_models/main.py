#!/usr/bin/env python

from zzz_cognition_msgs.msg import MapState


class MainDecision(object):
    def __init__(self, trajectory_planner=None):
        self._dynamic_map_buffer = None
        self._trajectory_planner = trajectory_planner
        self.during_restart = False # Stop planning during restart, clear buff.

    def receive_dynamic_map(self, dynamic_map):
        assert type(dynamic_map) == MapState
        self._dynamic_map_buffer = dynamic_map

    def update_trajectory(self, close_to_junction=10):
        if self._dynamic_map_buffer is None:
            return None
        dynamic_map = self._dynamic_map_buffer
        print("dynamic_map.model",dynamic_map.model)
        if dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP and dynamic_map.mmap.distance_to_junction > close_to_junction:
            self._trajectory_planner.clear_buff(dynamic_map)
            return None
        elif dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP:
            msg = self._trajectory_planner.initialize(dynamic_map)
            self.during_restart = False
            return None

        elif dynamic_map.ego_state.pose.pose.position.z < 0.5 and self.ego_not_at_start(dynamic_map) and self.during_restart == False:
            print("3333333",dynamic_map.ego_state.pose.pose.position.y)
            return self._trajectory_planner.trajectory_update(dynamic_map)


    def ego_not_at_start(self, dynamic_map):
        # # Town02 temp solution ,delete it!
        # if 133 < dynamic_map.ego_state.pose.pose.position.x < 140 and -230 < dynamic_map.ego_state.pose.pose.position.y < -210:
        #     return False

        # elif 100 < dynamic_map.ego_state.pose.pose.position.x < 110 and -190 < dynamic_map.ego_state.pose.pose.position.y < -180:
        #     return False
            
        # else:
        #     return True
        # Town05 highway temp solution ,delete it!
        if -110 < dynamic_map.ego_state.pose.pose.position.y < -70:
            return False

        elif 150 > dynamic_map.ego_state.pose.pose.position.y > 50:
            return False
            
        else:
            return True

    

    
