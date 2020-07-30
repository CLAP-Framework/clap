import os, sys
import io
import subprocess
from collections import deque
import tempfile
import math
import time

import rospy
import numpy as np
from zzz_navigation_msgs.msg import Lane, LanePoint, LaneBoundary, Map
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Path

class ShougangMap(object):
    def __init__(self, offset_x=0, offset_y=0):
        self._ego_vehicle_x = None # location in meters
        self._ego_vehicle_y = None

        self._offset_x = offset_x # map center offset in meters
        self._offset_y = offset_y

        self._ego_vehicle_x_buffer = 0.0
        self._ego_vehicle_y_buffer = 0.0

        self._reference_lane_list = deque(maxlen=20000)
        self._lanes = []
        self.load_lanes() # load lanes
        self._next_road_id_list = [1, 2, 3, 4, 5, 6, 7, 8, 0]

        self._current_road_id = -1
        self._next_road_id = -1

    def load_lanes(self):
        
        road1_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road1_0.txt',delimiter=',')
        road1_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road1_1.txt',delimiter=',')
        road2 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road2.txt',delimiter=',')
        road3 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road3.txt',delimiter=',')
        road4 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road4.txt',delimiter=',')
        road5 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road5.txt',delimiter=',')
        road6 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road6.txt',delimiter=',')
        road7_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road7_0.txt',delimiter=',')
        road7_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road7_1.txt',delimiter=',')

        road8_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road8_0.txt',delimiter=',')
        road8_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road8_1.txt',delimiter=',')
        road9_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road9_0.txt',delimiter=',')
        road9_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road9_1.txt',delimiter=',')
        
        self._lanes.append(dense_polyline2d(road1_0,1))
        self._lanes.append(dense_polyline2d(road2,1))
        self._lanes.append(dense_polyline2d(road3,1))
        self._lanes.append(dense_polyline2d(road4,1))
        self._lanes.append(dense_polyline2d(road5,1))
        self._lanes.append(dense_polyline2d(road6,1))
        self._lanes.append(dense_polyline2d(road7_0,1))
        self._lanes.append(dense_polyline2d(road8_0,1))
        self._lanes.append(dense_polyline2d(road9_0,1))

        self.road1_1 = dense_polyline2d(road1_1,1)
        self.road7_1 = dense_polyline2d(road7_1,1)
        self.road8_1 = dense_polyline2d(road8_1,1)
        self.road9_1 = dense_polyline2d(road9_1,1)

    def ego_road(self):

        dist_list = np.array([dist_from_point_to_polyline2d(
                self._ego_vehicle_x, self._ego_vehicle_y,
                lane) for lane in self._lanes])
        closest_lane, second_closest_lane = np.abs(dist_list[:, 0]).argsort()[:2]

        return closest_lane

    def receive_new_pose(self, x, y):
        self._ego_vehicle_x_buffer = x
        self._ego_vehicle_y_buffer = y

    def update(self):
        self._ego_vehicle_x = self._ego_vehicle_x_buffer
        self._ego_vehicle_y = self._ego_vehicle_y_buffer
        self.ego_road_id = self.ego_road()

        if self.should_update_static_map():
            self._current_road_id = self.ego_road_id
            self._next_road_id = self._next_road_id_list[self.ego_road_id]
            self.update_static_map()
            return self.static_local_map

        return None

    def should_update_static_map(self):
        '''
        Determine whether map updating is needed.
        '''
        return True

        if self._current_road_id < 0:
            return True
        
        if self._current_road_id == self.ego_road_id:
            return False
        
        if self._next_road_id == self.ego_road_id:
            return True

        return False

    def init_static_map(self):
        '''
        Generate null static map
        '''
        init_static_map = Map()
        init_static_map.in_junction = False
        init_static_map.target_lane_index = 0
        return init_static_map


    def update_static_map(self):
        ''' 
        Update information in the static map if current location changed dramatically
        '''
        map_x, map_y = self._ego_vehicle_x, self._ego_vehicle_y

        self.static_local_map = self.init_static_map()  ## Return this one
        self.update_lane_list(self.ego_road_id)
        self.update_next_road_list(self._next_road_id)

    def update_lane_list(self, road_id):
        '''
        Update lanes when a new road is encountered
        '''
        # map_x, map_y = self._ego_vehicle_x, self._ego_vehicle_y
        # Right is 0

        self.static_local_map.in_junction = False # lane change
        self.static_local_map.lanes.append(self.get_lane(self._lanes[road_id]))

        if road_id == (1-1):
            self.static_local_map.lanes.append(self.get_lane(self.road1_1))
            self.static_local_map.target_lane_index = 1
        
        if road_id == (7-1):
            self.static_local_map.lanes.append(self.get_lane(self.road7_1))

        if road_id == (8-1):
            self.static_local_map.lanes.append(self.get_lane(self.road8_1))

        if road_id == (9-1):
            self.static_local_map.target_lane_index = 1
            self.static_local_map.lanes.append(self.get_lane(self.road9_1))

        for i, lane in enumerate(self.static_local_map.lanes):
            if road_id == (1-1):
                lane.speed_limit = 35
                if i == 0:
                    lane.traffic_light_pos.append(298)
                if i == 1:
                    lane.traffic_light_pos.append(297)
                lane.traffic_light_pos.append(0)
            if road_id == (2-1):
                lane.speed_limit = 30
                lane.traffic_light_pos.append(0)
            if road_id == (3-1):
                lane.speed_limit = 20
                lane.traffic_light_pos.append(0)
            if road_id == (4-1):
                lane.speed_limit = 30
                lane.traffic_light_pos.append(0)
            if road_id == (5-1):
                lane.speed_limit = 30
                lane.traffic_light_pos.append(0)
            if road_id == (6-1):
                lane.speed_limit = 30
                lane.traffic_light_pos.append(0)
            if road_id == (7-1):
                lane.speed_limit = 35
                lane.traffic_light_pos.append(0)
            if road_id == (8-1):
                lane.speed_limit = 35
                lane.traffic_light_pos.append(0)
            if road_id == (9-1):
                lane.speed_limit = 35
                if i == 0:
                    lane.traffic_light_pos.append(330)
                if i == 1:
                    lane.traffic_light_pos.append(331)
                lane.traffic_light_pos.append(0)

    def update_next_road_list(self, next_road_id):

        self.static_local_map.next_lanes.append(self.get_lane(self._lanes[next_road_id]))

        if next_road_id == (1-1):
            self.static_local_map.next_lanes.append(self.get_lane(self.road1_1))

        if next_road_id == (8-1):
            self.static_local_map.next_lanes.append(self.get_lane(self.road8_1))

        if next_road_id == (9-1):
            self.static_local_map.next_lanes.append(self.get_lane(self.road9_1))
        
    def get_lane(self, central_points): # outside

        lane_wrapped = Lane()
        for wp in central_points: # TODO Consider ego pose where is the start and end point.
            point = LanePoint()
            x, y = wp[0], wp[1]   # Point XY
            #x, y = wp[1], wp[0]
            point.position.x = x
            point.position.y = y
            lane_wrapped.central_path_points.append(point)

        return lane_wrapped


