
import rospy
import numpy as np

from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_navigation_msgs.msg import Map, Lane
from zzz_navigation_msgs.utils import get_lane_array
from zzz_cognition_msgs.msg import MapState, LaneState, RoadObstacle
from zzz_cognition_msgs.utils import convert_tracking_box, default_msg as cognition_default
from zzz_perception_msgs.msg import TrackingBoxArray, TrafficLightDetection, TrafficLightDetectionArray
from zzz_common.geometry import dist_from_point_to_polyline2d
from zzz_common.dynamic_models import wrap_angle

from zzz_driver_msgs.utils import get_speed, get_yaw

class NearestLocator:
    def __init__(self):
        self._static_map = Map()
        self._static_map.in_junction = True
        self._static_map_lane_path_array = None
        self._dynamic_map = None
        self._surrounding_object_list = None
        self._ego_vehicle_state = None
        self._traffic_light_detection = None
        
        self._ego_vehicle_distance_to_lane_head = [] # distance from vehicle to lane start
        self._ego_vehicle_distance_to_lane_tail = [] # distance from vehicle to lane end

    @property
    def dynamic_map(self):
        return self._dynamic_map

    # ====== Data Receiver =======

    def receive_static_map(self, static_map):
        assert type(static_map) == Map

        self._static_map = static_map
        self._static_map_lane_path_array = get_lane_array(static_map.lanes)
        rospy.loginfo("Updated Local Static Map: lanes_num = %d, in_junction = %d, target_lane_index = %d",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._static_map.target_lane_index)

    def receive_object_list(self, object_list):
        assert type(object_list) == TrackingBoxArray
        if self._ego_vehicle_state != None:
            self._surrounding_object_list = convert_tracking_box(object_list, self._ego_vehicle_state)

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped
        self._ego_vehicle_state = state

    def receive_traffic_light_detection(self, detection):
        assert type(detection) == TrafficLightDetectionArray
        self._traffic_light_detection = detection

    # ====== Data Updator =======
    
    def update(self):
        self._dynamic_map = cognition_default(MapState)
        self._dynamic_map.header.frame_id = "map"
        self._dynamic_map.header.stamp = rospy.Time.now()

        # Create dynamic maps and add static map elements
        self._dynamic_map.ego_state = self._ego_vehicle_state.state
        if self._static_map.in_junction or len(self._static_map.lanes) == 0:
            rospy.logdebug("In junction due to state map report junction location")
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            self._dynamic_map.jmap.drivable_area = self._static_map.drivable_area
        else:
            self._dynamic_map.model = MapState.MODEL_MULTILANE_MAP
            for lane in self._static_map.lanes:
                dlane = cognition_default(LaneState)
                dlane.map_lane = lane
                self._dynamic_map.mmap.lanes.append(dlane)
            self._dynamic_map.mmap.target_lane_index = self._static_map.target_lane_index

        # Locate vehicles onto the junction map
        self.add_obstacles()
        self.locate_vehicles_in_junction()

        # Locate vehicles onto the multilane map
        if self._dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
            self.locate_ego_vehicle_in_lanes()
            self.locate_surrounding_vehicle_in_lanes()
            self.locate_stop_sign_in_lanes()
            self.locate_speed_limit_in_lanes()

        rospy.logdebug("Updated Dynamic Map: lanes_num = %d, in_junction = %d, ego_y = %d, distance_to_end = %f",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._dynamic_map.mmap.ego_lane_index, self._dynamic_map.mmap.distance_to_junction)

    # ========= For in lane =========

    def locate_ego_vehicle_in_lanes(self, lane_end_dist_thres=2, lane_dist_thres=5):
        if self._static_map_lane_path_array == None: # TODO: This should not happen 
            return

        dist_list = np.array([dist_from_point_to_polyline2d(
            self._ego_vehicle_state.state.pose.pose.position.x, self._ego_vehicle_state.state.pose.pose.position.y,
            lane, return_end_distance=True)
            for lane in self._static_map_lane_path_array])  
        dist_list = np.abs(dist_list)
        closest_lane = np.argmin(dist_list[:, 0])
        if dist_list[closest_lane, 0] > lane_dist_thres:
            rospy.logdebug("In junction due to far away from a lane")
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            return

        self._ego_vehicle_distance_to_lane_head = dist_list[:, 1]
        self._ego_vehicle_distance_to_lane_tail = dist_list[:, 2]
        
        if self._ego_vehicle_distance_to_lane_tail[closest_lane] <= lane_end_dist_thres:
            # Drive into junction, wait until next map # TODO: change the condition
            rospy.logdebug("In junction due to close to intersection")
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            return
        else:
            self._dynamic_map.model = MapState.MODEL_MULTILANE_MAP
            # TODO: Calculate continuous lane_index
            # self._dynamic_map.mmap.ego_lane_index = self._static_map.lanes[closest_lane].index
            self._dynamic_map.mmap.distance_to_junction = self._ego_vehicle_distance_to_lane_tail[closest_lane]
            # FIXME(frenet):For ego state
            self.locate_ego_vehicle_in_frenet()
           
    def locate_ego_vehicle_in_frenet(self, in_lane_thres = 0.9):

        # ego_mmap_x is always 0
        ego_mmap_x = 0

        # for ego_mmap_y
        dist_list = np.array([dist_from_point_to_polyline2d(
            self._ego_vehicle_state.state.pose.pose.position.x, self._ego_vehicle_state.state.pose.pose.position.y,
            lane, return_end_distance=True)
            for lane in self._static_map_lane_path_array])  
        dist_list = np.abs(dist_list)

        # Check if there's only two lanes
        if len(self._dynamic_map.mmap.lanes) < 2:
            closest_lane = second_closest_lane = 0
        else:
            closest_lane, second_closest_lane = dist_list[:, 0].argsort()[:2]

        closest_lane_dist, second_closest_lane_dist = dist_list[closest_lane, 0], dist_list[second_closest_lane, 0]
        closest_idx = int(dist_list[closest_lane, 1])
        closest_point = self._dynamic_map.mmap.lanes[closest_lane].map_lane.central_path_points[closest_idx]
        second_closest_idx = int(dist_list[second_closest_lane, 1])
        second_closest_point = self._dynamic_map.mmap.lanes[second_closest_lane].map_lane.central_path_points[second_closest_idx]

        ego_vehicle_driving_direction = get_yaw(self._ego_vehicle_state.state)
        lane_direction = closest_point.tangent
        d_theta = ego_vehicle_driving_direction - lane_direction
        d_theta = wrap_angle(d_theta)

        ego_speed = get_speed(self._ego_vehicle_state.state)

        # XXX: This is a naive method to detect whether the vehicle is outsize of the edge, could use signed distance to determine
        if abs(second_closest_lane_dist-closest_lane_dist) > ((closest_point.width/2)+(second_closest_point.width/2))*in_lane_thres:
            ego_mmap_y = closest_lane
            ego_mmap_vx = ego_speed
            ego_mmap_vy = 0
        else:
            a = closest_lane
            la = closest_lane_dist
            b = second_closest_lane
            lb = second_closest_lane_dist
            ego_mmap_y = (b*la+a*lb)/(lb+la)
            ego_mmap_vx = ego_speed*np.cos(d_theta)
            ego_mmap_vy = ego_speed*np.sin(d_theta)/closest_point.width

        self._dynamic_map.mmap.ego_lane_index = ego_mmap_y
        self._dynamic_map.mmap.ego_ffstate.s = ego_mmap_x
        self._dynamic_map.mmap.ego_ffstate.d = ego_mmap_y
        self._dynamic_map.mmap.ego_ffstate.vs = ego_mmap_vx
        self._dynamic_map.mmap.ego_ffstate.vd = ego_mmap_vy

    def locate_surrounding_vehicle_in_lanes(self, lane_dist_thres=3):
        surround_vehicles = self._surrounding_object_list # Prevent data update during processing XXX: use a better mechanism?
        lane_front_vehicle_list = [[] for _ in self._static_map.lanes]
        lane_rear_vehicle_list = [[] for _ in self._static_map.lanes]

        # TODO: separate vehicle and other objects?
        if self._surrounding_object_list is not None:
            for vehicle_idx, vehicle in enumerate(self._surrounding_object_list):
                dist_list = np.array([dist_from_point_to_polyline2d(vehicle.state.pose.pose.position.x, vehicle.state.pose.pose.position.y,
                    lane, return_end_distance=True)
                    for lane in self._static_map_lane_path_array])
                dist_list = np.abs(dist_list)
                closest_lane = np.argmin(dist_list[:, 0])
                
                # Determine if the vehicle is close to lane enough
                if dist_list[closest_lane, 0] > lane_dist_thres:
                    continue 
                if dist_list[closest_lane, 1] < self._ego_vehicle_distance_to_lane_head[closest_lane]:
                    # The vehicle is behind if its distance to lane start is smaller
                    lane_rear_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane, 1]))
                if dist_list[closest_lane, 2] < self._ego_vehicle_distance_to_lane_tail[closest_lane]:
                    # The vehicle is ahead if its distance to lane end is smaller
                    lane_front_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane, 2]))
        
        # Put the vehicles onto lanes
        for lane_id in range(len(self._static_map.lanes)):
            front_vehicles = np.array(lane_front_vehicle_list[lane_id])
            rear_vehicles = np.array(lane_rear_vehicle_list[lane_id])

            if len(front_vehicles) > 0:
                for vehicle_row in reversed(front_vehicles[:,1].argsort()):

                    # FIXME: Here only two vehicles are calculated because the speed is too slow
                    if len(self._dynamic_map.mmap.lanes[lane_id].front_vehicles) > 1:
                        continue

                    front_vehicle_idx = int(front_vehicles[vehicle_row, 0])
                    front_vehicle = surround_vehicles[front_vehicle_idx]
                    front_vehicle.ffstate.s = self._ego_vehicle_distance_to_lane_tail[lane_id] - front_vehicles[vehicle_row, 1]
                    front_vehicle.ffstate.d = self.vehicle_mmap_y(front_vehicle)
                    front_vehicle.ffstate.vs = get_speed(front_vehicle.state)
                    front_vehicle.ffstate.vd = 0
                    front_vehicle.behavior = self.predict_vehicle_behavior(front_vehicle)
                    self._dynamic_map.mmap.lanes[lane_id].front_vehicles.append(front_vehicle)
                    rospy.logdebug("Lane index: %d, Front vehicle id: %d, behavior: %d, x:%.1f, y:%.1f", 
                                    lane_id, front_vehicle.uid, front_vehicle.behavior,
                                    front_vehicle.state.pose.pose.position.x,front_vehicle.state.pose.pose.position.y)

            if len(rear_vehicles) > 0:
                for vehicle_row in reversed(rear_vehicles[:,1].argsort()):

                    # FIXME: Here only two vehicles are calculated because the speed is too slow
                    if len(self._dynamic_map.mmap.lanes[lane_id].rear_vehicles) > 1:
                        continue

                    rear_vehicle_idx = int(rear_vehicles[vehicle_row, 0])
                    rear_vehicle = surround_vehicles[rear_vehicle_idx]
                    rear_vehicle.ffstate.s = rear_vehicles[vehicle_row, 1] - self._ego_vehicle_distance_to_lane_head[lane_id] #negative value
                    rear_vehicle.ffstate.d = self.vehicle_mmap_y(rear_vehicle)
                    rear_vehicle.ffstate.vs = get_speed(rear_vehicle.state)
                    rear_vehicle.ffstate.vd = 0
                    rear_vehicle.behavior = self.predict_vehicle_behavior(rear_vehicle)
                    self._dynamic_map.mmap.lanes[lane_id].rear_vehicles.append(rear_vehicle)
                    rospy.logdebug("Lane index: %d, Rear vehicle id: %d, behavior: %d, x:%.1f, y:%.1f", 
                                    lane_id, rear_vehicle.uid, rear_vehicle.behavior, 
                                    rear_vehicle.state.pose.pose.position.x,rear_vehicle.state.pose.pose.position.y)

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

    # TODO: Move this function into separate prediction module
    def predict_vehicle_behavior(self, vehicle, lane_change_thres = 0.2):
        '''
        Detect the behaviors of surrounding vehicles
        '''

        dist_list = np.array([dist_from_point_to_polyline2d(vehicle.state.pose.pose.position.x, vehicle.state.pose.pose.position.y, lane)
            for lane in self._static_map_lane_path_array])
        dist_list = np.abs(dist_list)
        closest_lane = dist_list[:, 0].argsort()[0]
        closest_idx = int(dist_list[closest_lane, 1])
        closest_point = self._dynamic_map.mmap.lanes[closest_lane].map_lane.central_path_points[closest_idx]

        vehicle_driving_direction = get_yaw(vehicle.state)
        lane_direction = closest_point.tangent
        d_theta = vehicle_driving_direction - lane_direction
        d_theta = wrap_angle(d_theta)

        rospy.logdebug("id:%d, vehicle_direction:%.2f, lane_direction:%.2f",vehicle.uid,vehicle_driving_direction,lane_direction)
        if abs(d_theta) > lane_change_thres:
            if d_theta > 0:
                behavior = RoadObstacle.BEHAVIOR_MOVING_LEFT
            else:
                behavior = RoadObstacle.BEHAVIOR_MOVING_RIGHT
        else:
            behavior = RoadObstacle.BEHAVIOR_FOLLOW
        
        return behavior

    # TODO: Combine this into locate_surrounding_vehicle_in_lanes
    def vehicle_mmap_y(self, vehicle, in_lane_thres = 0.9,lane_dist_thres = 3):

        dist_list = np.array([dist_from_point_to_polyline2d(vehicle.state.pose.pose.position.x, vehicle.state.pose.pose.position.y, lane)
            for lane in self._static_map_lane_path_array])
        dist_list = np.abs(dist_list)
        
        # Check if there's only two lanes
        if len(self._dynamic_map.mmap.lanes) < 2:
            closest_lane = second_closest_lane = 0
        else:
            closest_lane, second_closest_lane = dist_list[:, 0].argsort()[:2]

        closest_lane_dist, second_closest_lane_dist = dist_list[closest_lane, 0], dist_list[second_closest_lane, 0]
        closest_idx = int(dist_list[closest_lane, 1])
        closest_point = self._dynamic_map.mmap.lanes[closest_lane].map_lane.central_path_points[closest_idx]
        second_closest_idx = int(dist_list[second_closest_lane, 1])
        second_closest_point = self._dynamic_map.mmap.lanes[second_closest_lane].map_lane.central_path_points[second_closest_idx]

        if closest_lane_dist > lane_dist_thres:
            mmap_y = -1
            return mmap_y

        # XXX: This is a naive method to detect whether the vehicle is outsize of the edge, could use signed distance to determine
        if abs(second_closest_lane_dist-closest_lane_dist) > ((closest_point.width/2)+(second_closest_point.width/2))*in_lane_thres:
            mmap_y = closest_lane
        else:
            a = closest_lane
            la = closest_lane_dist
            b = second_closest_lane
            lb = second_closest_lane_dist
            mmap_y = (b*la+a*lb)/(lb+la)
            
        return mmap_y
    
    # ========= For junction map =========

    def add_obstacles(self):
        self._dynamic_map.jmap.obstacles = []
        if self._surrounding_object_list == None:
            return
        for obj in self._surrounding_object_list:
            self._dynamic_map.jmap.obstacles.append(obj)

    def locate_vehicles_in_junction(self):
        
        if self._dynamic_map.model == MapState.MODEL_JUNCTION_MAP:
            return

        if self._static_map_lane_path_array == None:
            return

        for vehicle_idx, vehicle in enumerate(self._dynamic_map.jmap.obstacles):
            mmap_y = self.vehicle_mmap_y(vehicle)
            self._dynamic_map.jmap.obstacles[vehicle_idx].ffstate.d = mmap_y
