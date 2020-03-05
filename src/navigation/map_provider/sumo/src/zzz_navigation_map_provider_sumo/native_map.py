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

class NativeMap(object):
    def __init__(self, offset_x=0, offset_y=0):
        self._ego_vehicle_x = None # location in meters
        self._ego_vehicle_y = None

        self._offset_x = offset_x # map center offset in meters
        self._offset_y = offset_y

        self._ego_vehicle_x_buffer = 0.0
        self._ego_vehicle_y_buffer = 0.0

        self._reference_lane_list = deque(maxlen=20000)
        self._lanes = []  # 0-inner, 1-outer
        self.load_lanes() # load lanes

    def load_lanes(self):
        inner_path = os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/inner_loop.dat'
        outer_path = os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/outer_loop.dat'
        # inner 0, outer 1
        self._lanes.append(self.get_lane(np.loadtxt(inner_path)))
        self._lanes.append(self.get_lane(np.loadtxt(outer_path)))


    def receive_new_pose(self, x, y):
        self._ego_vehicle_x_buffer = x
        self._ego_vehicle_y_buffer = y


    def update(self):
        self._ego_vehicle_x = self._ego_vehicle_x_buffer
        self._ego_vehicle_y = self._ego_vehicle_y_buffer
        if self.should_update_static_map():
            self.update_static_map()
            return self.static_local_map

        return None


    def should_update_static_map(self):
        '''
        Determine whether map updating is needed.
        '''
        return True

    def init_static_map(self):
        '''
        Generate null static map
        '''
        init_static_map = Map()
        init_static_map.in_junction = True
        init_static_map.target_lane_index = -1
        return init_static_map


    def update_static_map(self):
        ''' 
        Update information in the static map if current location changed dramatically
        '''
        map_x, map_y = self._ego_vehicle_x, self._ego_vehicle_y

        self.static_local_map = self.init_static_map()  ## Return this one
        self.update_lane_list()
        

    def update_lane_list(self):
        '''
        Update lanes when a new road is encountered
        '''
        # map_x, map_y = self._ego_vehicle_x, self._ego_vehicle_y
        
        # Left is 0 
        self.static_local_map.lanes.append(self._lanes[0])
        self.static_local_map.lanes.append(self._lanes[1])


    def get_lane(self, central_points): # outside

        lane_wrapped = Lane()
        for wp in central_points: # TODO Consider ego pose where is the start and end point.
            point = LanePoint()
            x, y = wp[0], wp[1]   # Point XY
            point.position.x = x
            point.position.y = y
            lane_wrapped.central_path_points.append(point)

        return lane_wrapped


