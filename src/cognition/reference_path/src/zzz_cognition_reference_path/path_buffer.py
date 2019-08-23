
import rospy
import numpy as np
from collections import deque

from zzz_cognition_msgs.msg import MapState
from zzz_cognition_msgs.utils import default_msg
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_navigation_msgs.msg import LanePoint
from zzz_common.geometry import dist_from_point_to_polyline2d
from nav_msgs.msg import Path

class PathBuffer:
    def __init__(self, buffer_size=200):
        self._dynamic_map = default_msg(MapState)

        self._buffer_size = buffer_size
        self._reference_path = deque(maxlen=20000)
        self._reference_path_buffer = deque(maxlen=self._buffer_size)

        self._ego_vehicle_state = None

        self._judge_lane_change_threshold = 3

    def receive_static_map(self, map_input):
        assert type(map_input) == MapState

        self._dynamic_map = map_input

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped

        self._ego_vehicle_state = state.state

        # TODO: Move return to another function
        self.update_reference_path_buffer()
        return self._dynamic_map

    def receive_reference_path(self, reference_path):
        # TODO: Define a custom reference_path?
        assert type(reference_path) == Path

        self._reference_path.clear()
        for wp in reference_path.poses:
            self._reference_path.append((wp.pose.position.x, wp.pose.position.y))

    def update_reference_path_buffer(self):
        """
        Delete the passed point and add more point to the reference path
        """

        if self._reference_path_buffer:
            _, nearest_idx = dist_from_point_to_polyline2d(
                self._ego_vehicle_state.pose.pose.position.x,
                self._ego_vehicle_state.pose.pose.position.y,
                np.array(self._reference_path_buffer)
            )
            # Remove passed waypoints
            for _ in range(nearest_idx):
                self._reference_path_buffer.popleft()

        # Choose points from reference path to buffer
        while self._reference_path and len(self._reference_path_buffer) < self._buffer_size:
            wp = self._reference_path.popleft()
            self.lane_change_smoothen(wp) # Change this to a planning module
            self._reference_path_buffer.append(wp)

        # Put buffer into dynamic map
        for wp in self._reference_path_buffer:
            point = LanePoint()
            point.position.x = wp[0]
            point.position.y = wp[1]
            self._dynamic_map.jmap.reference_path.map_lane.central_path_points.append(point)
        
        self._dynamic_map.jmap.reference_path.map_lane.index = -1

        # Calculate vehicles on the reference path
        # TODO: find all vehicles near enough on reference_path
        front_vehicle = self.get_front_vehicle_on_reference_path()
        if front_vehicle is not None:
            self._dynamic_map.jmap.reference_path.front_vehicles = [front_vehicle]

        self._dynamic_map.jmap.reference_path.map_lane.speed_limit = 30

    def lane_change_smoothen(self, wp):
        """
        Avoid suddenly lane change
        """
        if self._reference_path_buffer:
            last_wp = np.array(self._reference_path_buffer[-1])
        else:
            return
        if np.linalg.norm(last_wp - wp) < self._judge_lane_change_threshold:
            return

        # Smoothen the reference path
        rospy.logdebug("Reference Path Smoothing")

        lane_change_distance = min(20, len(self._reference_path_buffer)) # TODO: Dynamic LC distance
        last_wp = wp
        first_wp = self._reference_path_buffer[-lane_change_distance]
        loc_xs = np.linspace(last_wp[0], first_wp[0], num = lane_change_distance+1)
        loc_ys = np.linspace(last_wp[1], first_wp[1], num = lane_change_distance+1)
        for i in range(1, lane_change_distance):
            self._reference_path_buffer[-1] = (loc_xs[i], loc_ys[i]) 

    def get_front_vehicle_on_reference_path(self, lane_dist_thres=2):
        """
        Get front vehicle on the reference path
        """

        front_vehicle = None
        nearest_dis = 200
        reference_path = np.array(self._reference_path_buffer)

        for vehicle in self._dynamic_map.jmap.obstacles:

            dist_list = np.linalg.norm(reference_path - [vehicle.state.pose.pose.position.x,
                vehicle.state.pose.pose.position.y], axis = 1)

            min_dis = np.min(dist_list)
            # min_idx = np.argmin(dist_list)
            if min_dis > lane_dist_thres:
                continue

            d = np.linalg.norm([
                vehicle.state.pose.pose.position.x - self._dynamic_map.ego_state.pose.pose.position.x,
                vehicle.state.pose.pose.position.y - self._dynamic_map.ego_state.pose.pose.position.y])

            if d < nearest_dis:
                front_vehicle = vehicle
                nearest_dis = d

        if front_vehicle is not None:
            rospy.logdebug("reference lane: front vehicle dis: %f", nearest_dis)

        return front_vehicle