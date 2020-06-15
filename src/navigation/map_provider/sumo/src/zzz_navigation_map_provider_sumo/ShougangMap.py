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

    def load_lanes(self):
        # inner_path = os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/inner_loop.dat'
        # outer_path = os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/outer_loop.dat'
        # # inner 1, outer 0
        # self._lanes.append(self.get_lane(np.loadtxt(outer_path, delimiter=',')))
        # self._lanes.append(self.get_lane(np.loadtxt(inner_path, delimiter=',')))

        road1_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road1_0.dat',delimiter=',')
        road1_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road1_1.dat',delimiter=',')
        road2 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road2.txt',delimiter=',')
        road3 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road3.txt',delimiter=',')
        road4 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road4.txt',delimiter=',')
        road5 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road5.txt',delimiter=',')
        road6 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road6.txt',delimiter=',')
        road7_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road7_0.txt',delimiter=',')
        road7_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road7_1.txt',delimiter=',')
        road8_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road8_0.dat',delimiter=',')
        road8_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road8_1.dat',delimiter=',')
        road9 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road9.txt',delimiter=',')
        road10_0 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road10_0.dat',delimiter=',')
        road10_1 = np.loadtxt(os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/road10_1.dat',delimiter=',')

        self._lanes.append(dense_polyline2d(road1_0,1))
        self._lanes.append(dense_polyline2d(road2,1))
        self._lanes.append(dense_polyline2d(road3,1))
        self._lanes.append(dense_polyline2d(road4,1))
        self._lanes.append(dense_polyline2d(road5,1))
        self._lanes.append(dense_polyline2d(road6,1))
        self._lanes.append(dense_polyline2d(road7_0,1))
        self._lanes.append(dense_polyline2d(road8_0,1))
        self._lanes.append(dense_polyline2d(road9,1))
        self._lanes.append(dense_polyline2d(road10_0,1))

        self.road1_1 = dense_polyline2d(road1_1,1)
        self.road7_1 = dense_polyline2d(road7_1,1)
        self.road8_1 = dense_polyline2d(road8_1,1)
        self.road10_1 = dense_polyline2d(road10_1,1)

        # self._lanes_inside.append(dense_polyline2d(np.array([[23.4,-91.234],[-36.214,-91.5]]),1))
        # self._lanes_inside.append(dense_polyline2d(np.array([[-50.942,-71.59],[-51.1,-10.56]]),1))
        # self._lanes_inside.append(dense_polyline2d(np.array([[-37.62,3.11],[15.03,3.37]]),1))
        # self._lanes_inside.append(dense_polyline2d(np.array([[31.8,-14.33],[33.52,-74.71]]),1))

        # self._lanes_outside.append(dense_polyline2d(np.array([[22.246,-94.584],[-37,-94.817]]),1))
        # self._lanes_outside.append(dense_polyline2d(np.array([[-55,-71.85],[-54.35,-10.114]]),1))
        # self._lanes_outside.append(dense_polyline2d(np.array([[-37.11,6.15],[15.03,6.16]]),1))
        # self._lanes_outside.append(dense_polyline2d(np.array([[34.76,-14.98],[36.53,-74.61]]),1))

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
        init_static_map.in_junction = False
        init_static_map.target_lane_index = 0
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

        self.static_local_map.in_junction = False # lane change
        ego_road = self.ego_road()
        self.static_local_map.lanes.append(self.get_lane(self._lanes[ego_road]))
        if ego_road == (1-1):
            self.static_local_map.lanes.append(self.get_lane(self.road1_1))
            self.static_local_map.target_lane_index = 1
        if ego_road == (7-1):
            self.static_local_map.lanes.append(self.get_lane(self.road7_1))

        if ego_road == (8-1):
            self.static_local_map.lanes.append(self.get_lane(self.road8_1))

        if ego_road == (10-1):
            self.static_local_map.target_lane_index = 1
            self.static_local_map.lanes.append(self.get_lane(self.road10_1))

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


