
import rospy
import numpy as np

from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_navigation_msgs.msg import Map, Lane
from zzz_navigation_msgs.utils import get_lane_array
from zzz_cognition_msgs.msg import MapState, LaneState, RoadObstacle
from zzz_cognition_msgs.utils import convert_tracking_box, default_msg as cognition_default
from zzz_perception_msgs.msg import TrackingBoxArray, DetectionBoxArray, ObjectSignals
from zzz_common.geometry import dist_from_point_to_polyline2d, wrap_angle
from zzz_common.kinematics import get_frenet_state

from zzz_driver_msgs.utils import get_speed, get_yaw

class NearestLocator:
    def __init__(self, lane_dist_thres=5):
        self._static_map = Map()
        self._static_map.in_junction = True
        self._static_map_lane_path_array = None
        self._static_map_lane_tangets = None
        self._dynamic_map = None
        self._surrounding_object_list = None
        self._surrounding_object_list_buffer = None
        self._ego_vehicle_state = None
        self._traffic_light_detection = None
        
        self._lane_dist_thres = lane_dist_thres

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
        self._static_map_lane_tangets = [[point.tangent for point in lane.central_path_points] for lane in static_map.lanes]
        rospy.loginfo("Updated Local Static Map: lanes_num = %d, in_junction = %d, target_lane_index = %d",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._static_map.target_lane_index)

    def receive_object_list(self, object_list):
        assert type(object_list) == TrackingBoxArray
        if self._ego_vehicle_state != None:
            self._surrounding_object_list_buffer = convert_tracking_box(object_list, self._ego_vehicle_state)

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped
        self._ego_vehicle_state = state

    def receive_traffic_light_detection(self, detection):
        assert type(detection) == DetectionBoxArray
        self._traffic_light_detection = detection

    # ====== Data Updator =======
    
    def update(self):
        self._dynamic_map = cognition_default(MapState)
        self._dynamic_map.header.frame_id = "map"
        self._dynamic_map.header.stamp = rospy.Time.now()

        # Update buffer information
        self._surrounding_object_list = self._surrounding_object_list_buffer

        # Create dynamic maps and add static map elements
        self._dynamic_map.ego_state = self._ego_vehicle_state.state
        if self._static_map.in_junction or len(self._static_map.lanes) == 0:
            rospy.logdebug("In junction due to static map report junction location")
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
        # TODO: Calculate frenet coordinate for objects in here or in put_buffer?
        self.locate_objects_in_junction()

        # Locate vehicles onto the multilane map
        if self._dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
            self.locate_ego_vehicle_in_lanes()
            self.locate_surrounding_objects_in_lanes()
            self.locate_stop_sign_in_lanes()
            self.locate_speed_limit_in_lanes()

        rospy.logdebug("Updated Dynamic Map: lanes_num = %d, in_junction = %d, lane_index = %d, distance_to_end = %f",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._dynamic_map.mmap.ego_lane_index, self._dynamic_map.mmap.distance_to_junction)

    # ========= For in lane =========

    def locate_object_in_lane(self, object_state, dist_list=None):
        '''
        Calculate (continuous) lane index for a object.
        Parameters: dist_list is the distance buffer. If not provided, it will be calculated
        '''

        if not dist_list:
            dist_list = np.array([dist_from_point_to_polyline2d(
                object_state.pose.pose.position.x,
                object_state.pose.pose.position.y,
                lane) for lane in self._static_map_lane_path_array])
        
        # Check if there's only two lanes
        if len(self._dynamic_map.mmap.lanes) < 2:
            closest_lane = second_closest_lane = 0
        else:
            closest_lane, second_closest_lane = np.abs(dist_list[:, 0]).argsort()[:2]

        # Signed distance from target to two closest lane
        closest_lane_dist, second_closest_lane_dist = dist_list[closest_lane, 0], dist_list[second_closest_lane, 0]

        if abs(closest_lane_dist) > self._lane_dist_thres:
            return -1 # TODO: return reasonable value

        # Judge whether the point is outside of lanes
        if closest_lane == second_closest_lane or closest_lane_dist * second_closest_lane_dist > 0:
            # The object is at left or right most
            return closest_lane
        else:
            # The object is between center line of lanes
            a, b = closest_lane, second_closest_lane
            la, lb = abs(closest_lane_dist), abs(second_closest_lane_dist)
            return (b*la + a*lb)/(lb + la)

    def locate_objects_in_junction(self):
        self._dynamic_map.jmap.obstacles = []
        if self._surrounding_object_list == None:
            return
        for obj in self._surrounding_object_list:
            if self._dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
                obj.lane_index = self.locate_object_in_lane(obj.state)
            else:
                obj.lane_index = -1
            self._dynamic_map.jmap.obstacles.append(obj)

    def locate_ego_vehicle_in_lanes(self, lane_end_dist_thres=2, lane_dist_thres=5):
        if self._static_map_lane_path_array == None: # TODO: This should not happen 
            return

        dist_list = np.array([dist_from_point_to_polyline2d(
            self._ego_vehicle_state.state.pose.pose.position.x, self._ego_vehicle_state.state.pose.pose.position.y,
            lane, return_end_distance=True)
            for lane in self._static_map_lane_path_array])  
        ego_lane_index = self.locate_object_in_lane(self._ego_vehicle_state.state)

        self._ego_vehicle_distance_to_lane_head = dist_list[:, 3]
        self._ego_vehicle_distance_to_lane_tail = dist_list[:, 4]
        if ego_lane_index < 0 or self._ego_vehicle_distance_to_lane_tail[int(ego_lane_index)] <= lane_end_dist_thres:
            # Drive into junction, wait until next map
            rospy.logdebug("In junction due to close to intersection")
            self._dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            # TODO: Calculate frenet coordinate here or in put_buffer?
            return
        else:
            self._dynamic_map.model = MapState.MODEL_MULTILANE_MAP
            self._dynamic_map.ego_ffstate = get_frenet_state(self._ego_vehicle_state, 
                self._static_map_lane_path_array[int(ego_lane_index)],
                self._static_map_lane_tangets[int(ego_lane_index)])
            self._dynamic_map.mmap.ego_lane_index = ego_lane_index
            self._dynamic_map.mmap.distance_to_junction = self._ego_vehicle_distance_to_lane_tail[int(ego_lane_index)]

    def locate_surrounding_objects_in_lanes(self, lane_dist_thres=3):
        surround_vehicles = self._surrounding_object_list # Prevent data update during processing XXX: use a better mechanism?
        lane_front_vehicle_list = [[] for _ in self._static_map.lanes]
        lane_rear_vehicle_list = [[] for _ in self._static_map.lanes]

        # TODO: separate vehicle and other objects?
        if self._surrounding_object_list is not None:
            for vehicle_idx, vehicle in enumerate(self._surrounding_object_list):
                dist_list = np.array([dist_from_point_to_polyline2d(
                    vehicle.state.pose.pose.position.x,
                    vehicle.state.pose.pose.position.y,
                    lane, return_end_distance=True)
                    for lane in self._static_map_lane_path_array])
                closest_lane = np.argmin(np.abs(dist_list[:, 0]))
                
                # Determine if the vehicle is close to lane enough
                if abs(dist_list[closest_lane, 0]) > lane_dist_thres:
                    continue 
                if dist_list[closest_lane, 3] < self._ego_vehicle_distance_to_lane_head[closest_lane]:
                    # The vehicle is behind if its distance to lane start is smaller
                    lane_rear_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane, 3]))
                if dist_list[closest_lane, 4] < self._ego_vehicle_distance_to_lane_tail[closest_lane]:
                    # The vehicle is ahead if its distance to lane end is smaller
                    lane_front_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane, 4]))
        
        # Put the vehicles onto lanes
        for lane_id in range(len(self._static_map.lanes)):
            front_vehicles = np.array(lane_front_vehicle_list[lane_id])
            rear_vehicles = np.array(lane_rear_vehicle_list[lane_id])

            if len(front_vehicles) > 0:
                # Descending sort front objects by distance to lane end
                for vehicle_row in reversed(front_vehicles[:,1].argsort()):
                    front_vehicle_idx = int(front_vehicles[vehicle_row, 0])
                    front_vehicle = surround_vehicles[front_vehicle_idx]
                    front_vehicle.ffstate = get_frenet_state(front_vehicle.state,
                        self._static_map_lane_path_array[lane_id],
                        self._static_map_lane_tangets[lane_id]
                    )
                    # Here we use relative frenet coordinate
                    front_vehicle.ffstate.s = self._ego_vehicle_distance_to_lane_tail[lane_id] - front_vehicles[vehicle_row, 1]
                    front_vehicle.behavior = self.predict_vehicle_behavior(front_vehicle)
                    self._dynamic_map.mmap.lanes[lane_id].front_vehicles.append(front_vehicle)
                    rospy.logdebug("Lane index: %d, Front vehicle id: %d, behavior: %d, x:%.1f, y:%.1f", 
                                    lane_id, front_vehicle.uid, front_vehicle.behavior,
                                    front_vehicle.state.pose.pose.position.x,front_vehicle.state.pose.pose.position.y)

            if len(rear_vehicles) > 0:
                # Descending sort rear objects by distance to lane end
                for vehicle_row in reversed(rear_vehicles[:,1].argsort()):
                    rear_vehicle_idx = int(rear_vehicles[vehicle_row, 0])
                    rear_vehicle = surround_vehicles[rear_vehicle_idx]
                    rear_vehicle.ffstate = get_frenet_state(rear_vehicle.state,
                        self._static_map_lane_path_array[lane_id],
                        self._static_map_lane_tangets[lane_id]
                    )
                    # Here we use relative frenet coordinate
                    rear_vehicle.ffstate.s = rear_vehicles[vehicle_row, 1] - self._ego_vehicle_distance_to_lane_head[lane_id] # negative value
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
                if lights[0].signal == ObjectSignals.TRAFFIC_LIGHT_RED:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                elif lights[0].signal == ObjectSignals.TRAFFIC_LIGHT_YELLOW:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_YIELD
                elif lights[0].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        elif len(lights) > 1 and len(lights) == total_lane_num:
            for i in range(total_lane_num):
                if lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_RED:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                elif lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_YELLOW:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_YIELD
                elif lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        elif len(lights) > 1 and len(lights) != total_lane_num:
            red = True
            for i in range(len(lights)):
                if lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    red = False
            for i in range(total_lane_num):
                if red:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                else:
                    self._dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        
    def locate_stop_sign_in_lanes(self):
        '''
        Put stop sign detections into lanes
        '''
        # TODO: Implement this
        pass

    def locate_speed_limit_in_lanes(self):
        '''
        Put stop sign detections into lanes
        '''
        # TODO(zhcao): Change the speed limit according to the map or the traffic sign(perception)
        # Now we set the multilane speed limit as 40 km/h.
        total_lane_num = len(self._static_map.lanes)
        for i in range(total_lane_num):
            self._dynamic_map.mmap.lanes[i].map_lane.speed_limit = 40

    # TODO(zyxin): Move this function into separate prediction module
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

