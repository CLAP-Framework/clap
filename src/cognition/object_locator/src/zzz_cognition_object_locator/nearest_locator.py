
import rospy
import numpy as np

from zzz_driver_msgs.msg import RigidBodyState
from zzz_navigation_msgs.msg import Map, Lane
from zzz_navigation_msgs.utils import get_lane_array
from zzz_cognition_msgs.msg import MapState, LaneState
from zzz_cognition_msgs.utils import default_msg as cognition_default
from zzz_perception_msgs.msg import TrackingBoxArray, TrafficLightDetection, TrafficLightDetectionArray
from zzz_common.geometry import dist_from_point_to_polyline, nearest_point_to_polyline

class NearestLocator:
    def __init__(self):
        self._static_map = Map()
        self._static_map.in_junction = True
        self._static_map_lane_path_array = None
        self._dynamic_map = None
        self._surrounding_object_list = None
        self._ego_vehicle_state = None
        self._traffic_light_detection = None
        
        self._ego_vehicle_distance_to_lane_head = 0 # distance from vehicle to lane start
        self._ego_vehicle_distance_to_lane_tail = 0 # distance from vehicle to lane end

    # ====== Data Receiver =======

    def receive_static_map(self, static_map):
        assert type(static_map) == Map

        self._static_map = static_map
        self._static_map_lane_path_array = get_lane_array(static_map.lanes)
        rospy.loginfo("Updated Local Static Map: lanes_num = %d, in_junction = %d, target_lane_index = %d",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._static_map.target_lane_index)

    def receive_object_list(self, object_list):
        assert type(object_list) == TrackingBoxArray
        self._surrounding_object_list = object_list

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyState
        self._ego_vehicle_state = state

    def receive_traffic_light_detection(self, detection):
        assert type(detection) == TrafficLightDetectionArray
        self._traffic_light_detection = detection

    # ====== Data Updator =======
    # TODO: edit following implementations
    
    def update(self):
        self._dynamic_map = cognition_default(MapState)
        self._dynamic_map.ego_state = self._ego_vehicle_state
        if self._static_map.in_junction or len(self._static_map.lanes) == 0:
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            self._dynamic_map.jmap.drivable_area = self._static_map.drivable_area
        else:
            self._dynamic_map.model = MapState.MODEL_MULTILANE_MAP
            for lane in self._static_map.lanes:
                dlane = cognition_default(LaneState)
                dlane.map_lane = lane
                dlane.stop_distance = float('inf') # XXX: add default constructor
                self._dynamic_map.mmap.lanes.append(dlane)
            self._dynamic_map.mmap.target_lane_index = self._static_map.target_lane_index

        if self._dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
            self.locate_ego_vehicle()
            self.locate_surrounding_vehicle_in_lanes()
            self.locate_stop_sign_in_lanes()
            self.locate_speed_limit_in_lanes()

        rospy.logdebug("Updated Dynamic Map: lanes_num = %d, in_junction = %d, ego_y = %d, distance_to_end = %f",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._dynamic_map.mmap.ego_lane_index, self._dynamic_map.mmap.distance_to_junction)

    """
    For in lane
    """

    def locate_ego_vehicle(self, lane_end_dist_thres=2, lane_dist_thres=5): 
        dist_list = np.array([dist_from_point_to_polyline(
            self._ego_vehicle_state.pose.pose.x, self._ego_vehicle_state.pose.pose.y, lane)
            for lane in self._static_map_lane_path_array])  
        closest_lane = np.argmin(dist_list[:, 0])

        if dist_list[closest_lane, 0] > lane_dist_thres:
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            return

        self._ego_vehicle_distance_to_lane_head = dist_list[closest_lane, 1]
        self._ego_vehicle_distance_to_lane_tail = dist_list[closest_lane, 2]
        
        if self._ego_vehicle_distance_to_lane_tail <= lane_end_dist_thres:
            # Drive into junction, wait until next map # TODO: change the condition
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            return
        else:
            self._dynamic_map.model = MapState.MODEL_MULTILANE_MAP
            self._dynamic_map.mmap.ego_lane_index = self._static_map.lanes[closest_lane].index
            self._dynamic_map.mmap.distance_to_junction = self._ego_vehicle_distance_to_lane_tail

    def locate_surrounding_vehicle_in_lanes(self, lane_dist_thres=2):
        lane_front_vehicle_list = [[] for _ in self._static_map.lanes]
        lane_rear_vehicle_list = [[] for _ in self._static_map.lanes]

        # TODO: separate vehicle and other objects?
        for vehicle_idx, vehicle in enumerate(self._surrounding_object_list):
            dist_list = np.array([dist_from_point_to_polyline(vehicle.obstacle_pos_x, vehicle.obstacle_pos_y, lane)
                for lane in self._static_map_lane_path_array])
            dist_list = np.abs(dist_list)
            closest_lane = np.argmin(dist_list[:, 0])

            # Determine if the vehicle is close to lane enough
            if dist_list[closest_lane, 0] > lane_dist_thres:
                continue 
            if dist_list[closest_lane, 1] < self._ego_vehicle_distance_to_lane_head:
                # The vehicle is behind if its distance to lane start is smaller
                lane_rear_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane,1]))
            if dist_list[closest_lane, 2] < self._ego_vehicle_distance_to_lane_tail:
                # The vehicle is ahead if its distance to lane end is smaller
                lane_front_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane,2]))
        
        for lane_id in range(len(self._static_map.lanes)):
            front_vehicles = np.array(lane_front_vehicle_list[lane_id])
            rear_vehicles = np.array(lane_rear_vehicle_list[lane_id])
            if len(front_vehicles) > 0:
                # Select vehicle ahead with maximum distance to lane start as front vehicle
                front_vehicle_idx = int(front_vehicles[np.argmax(front_vehicles[:,1]), 0])
                self._dynamic_map.mmap.lanes[lane_id].have_front_vehicle = True
                self._dynamic_map.mmap.lanes[lane_id].front_vehicle = self._surrounding_object_list[front_vehicle_idx]
                rospy.logdebug("Lane index: %d, Front vehicle id: %d", lane_id, self._dynamic_map.mmap.lanes[lane_id].front_vehicle.uid)                
            else:
                self._dynamic_map.mmap.lanes[lane_id].have_front_vehicle = False


            if len(rear_vehicles) > 0:
                # Select vehicle behine with maximum distance to lane start as rear vehicle
                rear_vehicle_idx  = int(rear_vehicles [np.argmax(rear_vehicles[:,1]), 0])
                self._dynamic_map.mmap.lanes[lane_id].have_rear_vehicle = True
                self._dynamic_map.mmap.lanes[lane_id].rear_vehicle  = self._surrounding_object_list[rear_vehicle_idx]
                rospy.logdebug("Lane index: %d, Rear vehicle id: %d", lane_id, self._dynamic_map.mmap.lanes[lane_id].rear_vehicle.uid) 
            else:
                self._dynamic_map.mmap.lanes[lane_id].have_rear_vehicle = False

    def locate_traffic_light_in_lanes(self):
        # TODO: Currently it's a very simple rule to locate the traffic lights
        if self._traffic_light_detection is None:
            return
        lights = self._traffic_light_detection.detections

        total_lane_num = len(self._static_map.lanes)
        if len(lights) == 1:
            for i in range(total_lane_num):
                if lights[0].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_RED:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                elif lights[0].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_YELLOW:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_YIELD
                elif lights[0].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_GREEN:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_THRU
        elif len(lights) > 1 and len(lights) == total_lane_num:
            for i in range(total_lane_num):
                if lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_RED:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_STOP
                elif lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_YELLOW:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_YIELD
                elif lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_GREEN:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_THRU
        elif len(lights) > 1 and len(lights) != total_lane_num:
            red = True
            for i in range(len(lights)):
                if lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_GREEN:
                    red = False
            for i in range(total_lane_num):
                if red:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_STOP
                else:
                    self._dynamic_map.mmap.lanes[i].traffic_light_state = Lane.STOP_STATE_THRU
        
    def locate_stop_sign_in_lanes(self):
        '''
        Put stop sign detections into lanes
        '''
        pass

    def locate_speed_limit_in_lanes(self):
        '''
        Put stop sign detections into lanes
        '''
        pass
