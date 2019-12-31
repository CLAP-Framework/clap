
import rospy
import numpy as np
from collections import deque
from addict import Dict as edict

from zzz_cognition_msgs.msg import MapState
from zzz_cognition_msgs.utils import default_msg
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_navigation_msgs.msg import LanePoint
from zzz_common.geometry import dist_from_point_to_polyline2d
from nav_msgs.msg import Path

class PathBuffer:
    def __init__(self, buffer_size=200):
        self._buffer_size = buffer_size

        # Buffer for updating states
        self._static_map_buffer = None
        self._ego_vehicle_state_buffer = None
        self._reference_path_buffer = None
        self._reference_path_segment = deque(maxlen=buffer_size)
        self._reference_path_changed = False # Use this flag to avoid race condition on self._reference_path_segment

        self._judge_lane_change_threshold = 3

        self._rerouting_trigger = None
        self._rerouting_sent = False

    def set_rerouting_trigger(self, trigger):
        self._rerouting_trigger = trigger

    def receive_static_map(self, map_input):
        assert type(map_input) == MapState
        self._static_map_buffer = map_input
        rospy.logdebug("Updating local dynamic map")

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped
        self._ego_vehicle_state_buffer = state

    def receive_reference_path(self, reference_path):
        # TODO: Define a custom reference_path?
        assert type(reference_path) == Path

        # Here reference path is appended reversely in order to easy move points in a FIFO way
        self._reference_path_buffer = [(waypoint.pose.position.x, waypoint.pose.position.y) 
                                        for waypoint in reversed(reference_path.poses)]
        self._reference_path_changed = True
        rospy.loginfo("Received reference path, length:%d", len(reference_path.poses))

    def update(self,
               required_reference_path_length = 10,
               front_vehicle_avoidance_require_thres = 2,
               remained_passed_point = 5
        ):
        """
        Delete the passed point and add more point to the reference path
        """

        # Load states
        if not self._reference_path_buffer or not self._ego_vehicle_state_buffer:
            return None
        tstates = edict()
        tstates.dynamic_map = self._static_map_buffer or default_msg(MapState)
        tstates.ego_state = self._ego_vehicle_state_buffer.state
        dynamic_map = tstates.dynamic_map # for easy access
        ego_state = tstates.ego_state # for easy access

        # Process segment clear request
        if self._reference_path_changed:
            self._reference_path_segment.clear()
            self._reference_path_changed = False
        tstates.reference_path = self._reference_path_buffer
        reference_path = tstates.reference_path # for easy access

        # Remove passed waypoints
        if len(self._reference_path_segment) > 1:
            _, nearest_idx, _ = dist_from_point_to_polyline2d(
                ego_state.pose.pose.position.x,
                ego_state.pose.pose.position.y,
                np.array(self._reference_path_segment)
            )

            for _ in range(nearest_idx - remained_passed_point):
                removed_point = self._reference_path_segment.popleft()
                rospy.logdebug("removed waypoint: %s, remaining count: %d", str(removed_point), len(reference_path))

        # Current reference path is too short, require a new reference path
        if len(reference_path) < required_reference_path_length and not self._rerouting_sent:
            if not self._rerouting_trigger:
                self._rerouting_trigger()
                self._rerouting_sent = True
        else:
            self._rerouting_sent = False # reset flag

        # Choose points from reference path to buffer
        while reference_path and len(self._reference_path_segment) < self._buffer_size:
            wp = reference_path.pop() # notice that the points are inserted reversely
            # self.lane_change_smoothen(wp) # TODO(zhcao): find some bugs in this function, also change this to a planning module
            self._reference_path_segment.append(wp)

        # Put buffer into dynamic map
        for wp in self._reference_path_segment:
            point = LanePoint()
            point.position.x = wp[0]
            point.position.y = wp[1]
            dynamic_map.jmap.reference_path.map_lane.central_path_points.append(point)
        
        dynamic_map.jmap.reference_path.map_lane.index = -1

        # Calculate vehicles on the reference path 
        # TODO: find all vehicles near enough on reference_path
        if len(dynamic_map.jmap.reference_path.map_lane.central_path_points) > front_vehicle_avoidance_require_thres:
            front_vehicle = self.get_front_vehicle_on_reference_path(tstates)
            if front_vehicle is not None:
                dynamic_map.jmap.reference_path.front_vehicles = [front_vehicle]

        # TODO: read or detect speed limit
        # FIXME: These parameters are changed for experiment
        dynamic_map.jmap.reference_path.map_lane.speed_limit = 54
        return dynamic_map

    def lane_change_smoothen(self, wp):
        """
        Avoid suddenly lane change
        """
        if self._reference_path_segment:
            last_wp = np.array(self._reference_path_segment[-1])
        else:
            return
        if np.linalg.norm(last_wp - wp) < self._judge_lane_change_threshold:
            return

        # Smoothen the reference path
        rospy.logdebug("Reference Path Smoothing")

        lane_change_distance = min(20, len(self._reference_path_segment)) # TODO: Dynamic LC distance
        last_wp = wp
        first_wp = self._reference_path_segment[-lane_change_distance]
        loc_xs = np.linspace(last_wp[0], first_wp[0], num = lane_change_distance+1)
        loc_ys = np.linspace(last_wp[1], first_wp[1], num = lane_change_distance+1)
        for i in range(1, lane_change_distance):
            self._reference_path_segment[-1] = (loc_xs[i], loc_ys[i]) 

    def get_front_vehicle_on_reference_path(self, tstates, lane_dist_thres=2):
        """
        Get front vehicle on the reference path
        """

        front_vehicle = None
        nearest_dis = float('inf')
        reference_path = np.array(self._reference_path_segment)

        for vehicle in tstates.dynamic_map.jmap.obstacles:

            dist_list = np.linalg.norm(reference_path - [
                    vehicle.state.pose.pose.position.x,
                    vehicle.state.pose.pose.position.y
                ], axis = 1)

            min_dis = np.min(dist_list)
            if min_dis > lane_dist_thres:
                continue

            d = np.linalg.norm([
                vehicle.state.pose.pose.position.x - tstates.ego_state.pose.pose.position.x,
                vehicle.state.pose.pose.position.y - tstates.ego_state.pose.pose.position.y
            ])

            if d < nearest_dis:
                front_vehicle = vehicle
                nearest_dis = d

        if front_vehicle is not None:
            rospy.loginfo("front vehicle pos {}-{}, ego pos {}-{}, front distance {}".format(
                vehicle.state.pose.pose.position.x, vehicle.state.pose.pose.position.y,
                tstates.ego_state.pose.pose.position.x, tstates.ego_state.pose.pose.position.y, nearest_dis))

        return front_vehicle
