import numpy as np
import rospy
from easydict import EasyDict as edict
from threading import Lock
import copy
import math

from zzz_cognition_msgs.msg import LaneState, MapState, RoadObstacle
from zzz_cognition_msgs.utils import convert_tracking_box
from zzz_cognition_msgs.utils import default_msg as cognition_default
from zzz_common.geometry import dist_from_point_to_polyline2d, wrap_angle
from zzz_common.kinematics import get_frenet_state
from zzz_common.utils import quaternion_to_eular, calc_vehicle_corners
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_driver_msgs.utils import get_speed, get_yaw
from zzz_navigation_msgs.msg import Lane, Map
from zzz_navigation_msgs.utils import get_lane_array, default_msg as navigation_default
from zzz_perception_msgs.msg import (DetectionBoxArray, DetectionBox, ObjectSignals,
                                     TrackingBoxArray)
from zzz_visualization_rviz_box_visualizer.utils import RvizVisualizer

class NearestLocator:
    def __init__(self, lane_dist_thres=5): 
        
        self._static_map_lock = Lock()
        self._static_map_buffer = None

        self._ego_vehicle_state_lock = Lock()
        self._ego_vehicle_state_buffer = None

        self._surrounding_object_list_lock = Lock()
        self._surrounding_object_list_buffer = None

        self._traffic_light_detection_lock = Lock()
        self._traffic_light_detection_buffer = None
        
        self._lane_dist_thres = lane_dist_thres

        # These two are buffers for computation
        self._ego_vehicle_distance_to_lane_head = [] # distance from vehicle to lane start
        self._ego_vehicle_distance_to_lane_tail = [] # distance from vehicle to lane end

        self.nearest_front_vehicle_marker = None
        self.corner_to_lane_marker = None

    # ====== Data Receiver =======

    def receive_static_map(self, static_map):
        assert type(static_map) == Map
        with self._static_map_lock:
            self._static_map_buffer = static_map

    def receive_object_list(self, object_list):
        assert type(object_list) == TrackingBoxArray
        with self._surrounding_object_list_lock:
            if self._ego_vehicle_state_buffer != None:
                self._surrounding_object_list_buffer = convert_tracking_box(object_list, self._ego_vehicle_state_buffer)

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped
        with self._ego_vehicle_state_lock:
            self._ego_vehicle_state_buffer = state

    def receive_traffic_light_detection(self, detection):
        assert type(detection) == DetectionBoxArray
        with self._traffic_light_detection_lock:
            self._traffic_light_detection_buffer = detection

    # ====== Data Updator =======
    
    def update(self):
        # All transient states collected for this update:
        # - ego_state
        # - static_map
        # - dynamic_map
        # - static_map_lane_path_array
        # - static_map_lane_tangets
        # - surrounding_object_list
        tstates = edict()
        # Skip if not ready
        if not self._ego_vehicle_state_buffer:
            rospy.logwarn("Cognition: not receive ego pose")
            return None
        with self._ego_vehicle_state_lock:
            tstates.ego_state = copy.deepcopy(self._ego_vehicle_state_buffer) 

        # Update buffer information
        tstates.surrounding_object_list = copy.deepcopy(self._surrounding_object_list_buffer or [])

        tstates.static_map = copy.deepcopy(self._static_map_buffer or navigation_default(Map)) 
        static_map = tstates.static_map # for easier access
        tstates.static_map_lane_path_array = get_lane_array(tstates.static_map.lanes)
        tstates.static_map_lane_tangets = [[point.tangent for point in lane.central_path_points] for lane in tstates.static_map.lanes]
        tstates.dynamic_map = cognition_default(MapState)
        tstates.traffic_light_detection = self._traffic_light_detection_buffer
        dynamic_map = tstates.dynamic_map # for easier access

        # Create dynamic maps and add static map elements
        dynamic_map.header.frame_id = "map"
        dynamic_map.header.stamp = rospy.Time.now()
        dynamic_map.ego_state = tstates.ego_state.state

        if static_map.in_junction or len(static_map.lanes) == 0:
            rospy.logdebug("Cognition: In junction due to static map report junction location")
            dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            dynamic_map.jmap.drivable_area = static_map.drivable_area
        else:
            dynamic_map.model = MapState.MODEL_MULTILANE_MAP
            for lane in static_map.lanes:
                dlane = cognition_default(LaneState)
                dlane.map_lane = lane
                dynamic_map.mmap.lanes.append(dlane)
            dynamic_map.mmap.exit_lane_index = copy.deepcopy(static_map.exit_lane_index)

        # Locate vehicles onto the junction map
        # TODO: Calculate frenet coordinate for objects in here or in put_buffer?
        self.locate_objects_in_junction(tstates)

        # Locate vehicles onto the multilane map
        if dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
            self.locate_traffic_light_in_lanes(tstates)
            self.locate_ego_vehicle_in_lanes(tstates)
        
        if tstates.dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
            self.locate_surrounding_objects_in_lanes(tstates)
            self.locate_stop_sign_in_lanes(tstates)
            self.locate_speed_limit_in_lanes(tstates)

        if tstates.dynamic_map.model == MapState.MODEL_JUNCTION_MAP:
            # clean traffic lights
            self._traffic_light_detection_buffer = None

        return dynamic_map

    # ========= For in lane =========

    def locate_object_in_lane(self, object_state, tstates, dist_list=None):
        '''
        Calculate (continuous) lane index for a object.
        Parameters: dist_list is the distance buffer. If not provided, it will be calculated
        '''

        if not dist_list:
            dist_list = np.array([dist_from_point_to_polyline2d(
                object_state.pose.pose.position.x,
                object_state.pose.pose.position.y,
                lane) for lane in tstates.static_map_lane_path_array])
        
        # Check if there's only two lanes
        if len(tstates.dynamic_map.mmap.lanes) < 2:
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

    def locate_objects_in_junction(self, tstates, danger_area = 80):
        tstates.dynamic_map.jmap.obstacles = []
               
        for obj in tstates.surrounding_object_list:
            dist_to_ego = math.sqrt(math.pow((obj.state.pose.pose.position.x - tstates.ego_state.state.pose.pose.position.x),2) 
                + math.pow((obj.state.pose.pose.position.y - tstates.ego_state.state.pose.pose.position.y),2))

            if dist_to_ego > danger_area:
                continue

            if tstates.dynamic_map.model == MapState.MODEL_MULTILANE_MAP:
                obj.lane_index = self.locate_object_in_lane(obj.state, tstates)
            else:
                obj.lane_index = -1
            tstates.dynamic_map.jmap.obstacles.append(obj)

    # TODO: adjust lane_end_dist_thres to class variable
    def locate_ego_vehicle_in_lanes(self, tstates, 
                            lane_end_dist_thres=15,
                            lane_head_thres = 4, 
                            lane_dist_thres = 5):
        dist_list = np.array([dist_from_point_to_polyline2d(
            tstates.ego_state.state.pose.pose.position.x, tstates.ego_state.state.pose.pose.position.y,
            lane, return_end_distance=True)
            for lane in tstates.static_map_lane_path_array])

        ego_lane_index = self.locate_object_in_lane(tstates.ego_state.state, tstates)
        ego_lane_index_rounded = int(round(ego_lane_index))

        if ego_lane_index_rounded < 0 or ego_lane_index_rounded > len(tstates.static_map_lane_path_array) - 1:
            tstates.dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            rospy.logwarn("Cognition: Ego_lane_index_error")
            return
        
        self._ego_vehicle_distance_to_lane_head = dist_list[:, 3]
        self._ego_vehicle_distance_to_lane_tail = dist_list[:, 4]

        for i, lane in enumerate(tstates.dynamic_map.mmap.lanes):
            lane.ego_dis_to_lane_head = dist_list[i,3]
            lane.ego_dis_to_lane_tail = dist_list[i,4]
        
        if (self._ego_vehicle_distance_to_lane_tail[ego_lane_index_rounded] <= lane_end_dist_thres 
                and tstates.dynamic_map.mmap.lanes[ego_lane_index_rounded].map_lane.stop_state == Lane.STOP_STATE_THRU):
            # Drive into junction, wait until next map
            rospy.logdebug("Cognition: Ego vehicle close to intersection, ego_lane_index = %f, dist_to_lane_tail = %f", ego_lane_index, self._ego_vehicle_distance_to_lane_tail[int(ego_lane_index)])
            tstates.dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            # TODO: Calculate frenet coordinate here or in put_buffer?
            return
        
        if (self._ego_vehicle_distance_to_lane_head[int(ego_lane_index)] <= lane_head_thres):
            rospy.logdebug("Cognition: Ego vehicle close to next lanes, ego_lane_index = %f, dist_to_lane_head = %f", ego_lane_index, self._ego_vehicle_distance_to_lane_head[int(ego_lane_index)])
            tstates.dynamic_map.model = MapState.MODEL_JUNCTION_MAP
            # TODO: adjust this for prepare lane decision
            tstates.dynamic_map.jmap.distance_to_lanes = dist_list[ego_lane_index_rounded,0]
            return

        tstates.dynamic_map.model = MapState.MODEL_MULTILANE_MAP
        tstates.dynamic_map.ego_ffstate = get_frenet_state(tstates.ego_state, 
            tstates.static_map_lane_path_array[ego_lane_index_rounded],
            tstates.static_map_lane_tangets[ego_lane_index_rounded])
        tstates.dynamic_map.mmap.ego_lane_index = ego_lane_index
        tstates.dynamic_map.mmap.distance_to_junction = self._ego_vehicle_distance_to_lane_tail[ego_lane_index_rounded]

    def locate_surrounding_objects_in_lanes(self, tstates, lane_width=3):
        lane_front_vehicle_list = [[] for _ in tstates.static_map.lanes]
        lane_rear_vehicle_list = [[] for _ in tstates.static_map.lanes]

        if tstates.surrounding_object_list is not None:

            ego_v = get_speed(tstates.ego_state.state)

            for vehicle_idx, vehicle in enumerate(tstates.surrounding_object_list):
                # TODO: separate vehicle and other objects?
                # if vehicle.cls.classid == vehicle.cls.HUMAN and ego_v < 20/3.6:
                #     continue

                roll, pitch, yaw = quaternion_to_eular(vehicle.state.pose.pose.orientation.x,
                                                       vehicle.state.pose.pose.orientation.y,
                                                       vehicle.state.pose.pose.orientation.z,
                                                       vehicle.state.pose.pose.orientation.w)
                corner_mat = calc_vehicle_corners(vehicle.state.pose.pose.position.x,
                                                  vehicle.state.pose.pose.position.y,
                                                  vehicle.dimension.length_x,
                                                  vehicle.dimension.length_y,
                                                  yaw)
                dist_to_lanes_list = []
                for lane in tstates.static_map_lane_path_array:
                    corner_to_lane_dist = np.array([dist_from_point_to_polyline2d(
                        corner_mat[0, i],
                        corner_mat[1, i],
                        lane, return_end_distance=True) for i in range(4)])
                    # find closest corner point to current lane
                    min_dist_to_current_lane = np.argmin(np.abs(corner_to_lane_dist[:, 0]))
                    # distance from this closest corner point to current lane is taken as the minimum distance to lane
                    dist_to_lanes_list.append(corner_to_lane_dist[min_dist_to_current_lane])

                # compare the minimun corner distance and get the closest lane index
                dist_to_lanes_list = np.array(dist_to_lanes_list)
                closest_lane = np.argmin(np.abs(dist_to_lanes_list[:, 0]))

                # Determine if the vehicle is close to lane enough
                if abs(dist_to_lanes_list[closest_lane, 0]) > lane_width*0.5:
                    continue
                if dist_to_lanes_list[closest_lane, 3] < self._ego_vehicle_distance_to_lane_head[closest_lane]:
                    # The vehicle is behind if its distance to lane start is smaller
                    lane_rear_vehicle_list[closest_lane].append((vehicle_idx, dist_to_lanes_list[closest_lane, 3]))
                if dist_to_lanes_list[closest_lane, 4] < self._ego_vehicle_distance_to_lane_tail[closest_lane]:
                    # The vehicle is ahead if its distance to lane end is smaller
                    lane_front_vehicle_list[closest_lane].append((vehicle_idx, dist_to_lanes_list[closest_lane, 4]))
        
        # Put the vehicles onto lanes
        for lane_id in range(len(tstates.static_map.lanes)):
            front_vehicles = np.array(lane_front_vehicle_list[lane_id])
            rear_vehicles = np.array(lane_rear_vehicle_list[lane_id])

            if len(front_vehicles) > 0:
                # Descending sort front objects by distance to lane end
                for vehicle_row in reversed(front_vehicles[:,1].argsort()):
                    front_vehicle_idx = int(front_vehicles[vehicle_row, 0])
                    front_vehicle = tstates.surrounding_object_list[front_vehicle_idx]
                    front_vehicle.ffstate = get_frenet_state(front_vehicle.state,
                        tstates.static_map_lane_path_array[lane_id],
                        tstates.static_map_lane_tangets[lane_id]
                    )
                    # Here we use relative frenet coordinate
                    front_vehicle.ffstate.s = self._ego_vehicle_distance_to_lane_tail[lane_id] - front_vehicles[vehicle_row, 1]
                    front_vehicle.behavior = self.predict_vehicle_behavior(front_vehicle, tstates)
                    tstates.dynamic_map.mmap.lanes[lane_id].front_vehicles.append(front_vehicle)
                
                front_vehicle = tstates.dynamic_map.mmap.lanes[lane_id].front_vehicles[0]

                # from here to logdebug is for visulization
                self.nearest_front_vehicle_marker = RvizVisualizer.draw_words('FRONT\nVEHICLE',
                                                                              front_vehicle.state.pose.pose.position.x,
                                                                              front_vehicle.state.pose.pose.position.y,
                                                                              scale=1)
                roll, pitch, yaw = quaternion_to_eular(front_vehicle.state.pose.pose.orientation.x,
                                                       front_vehicle.state.pose.pose.orientation.y,
                                                       front_vehicle.state.pose.pose.orientation.z,
                                                       front_vehicle.state.pose.pose.orientation.w)
                corner_mat = calc_vehicle_corners(front_vehicle.state.pose.pose.position.x,
                                                  front_vehicle.state.pose.pose.position.y,
                                                  front_vehicle.dimension.length_x,
                                                  front_vehicle.dimension.length_y,
                                                  yaw)
                corner_to_lane_dist = np.array([dist_from_point_to_polyline2d(
                    corner_mat[0, i],
                    corner_mat[1, i],
                    tstates.static_map_lane_path_array[lane_id], return_end_distance=True) for i in range(4)])
                min_dist_to_current_lane = np.argmin(np.abs(corner_to_lane_dist[:, 0]))
                corner_point = (corner_mat[0, min_dist_to_current_lane], corner_mat[1, min_dist_to_current_lane])
                closet_point_inlane_index = int(corner_to_lane_dist[min_dist_to_current_lane, 1])
                lane_point = tstates.static_map_lane_path_array[lane_id][closet_point_inlane_index].tolist()
                self.corner_to_lane_marker = RvizVisualizer.draw_two_points_line(corner_point, lane_point, width=0.1)

                # rospy.logdebug("Lane index: %d, Front vehicle id: %d, behavior: %d, x:%.1f, y:%.1f, d:%.1f",
                #                 lane_id, front_vehicle.uid, front_vehicle.behavior,
                #                 front_vehicle.state.pose.pose.position.x,front_vehicle.state.pose.pose.position.y,
                #                 front_vehicle.ffstate.s)

            if len(rear_vehicles) > 0:
                # Descending sort rear objects by distance to lane end
                for vehicle_row in reversed(rear_vehicles[:,1].argsort()):
                    rear_vehicle_idx = int(rear_vehicles[vehicle_row, 0])
                    rear_vehicle = tstates.surrounding_object_list[rear_vehicle_idx]
                    rear_vehicle.ffstate = get_frenet_state(rear_vehicle.state,
                        tstates.static_map_lane_path_array[lane_id],
                        tstates.static_map_lane_tangets[lane_id]
                    )
                    # Here we use relative frenet coordinate
                    rear_vehicle.ffstate.s = rear_vehicles[vehicle_row, 1] - self._ego_vehicle_distance_to_lane_head[lane_id] # negative value
                    rear_vehicle.behavior = self.predict_vehicle_behavior(rear_vehicle, tstates)
                    tstates.dynamic_map.mmap.lanes[lane_id].rear_vehicles.append(rear_vehicle)
                
                rear_vehicle = tstates.dynamic_map.mmap.lanes[lane_id].rear_vehicles[0]

                # rospy.logdebug("Lane index: %d, Rear vehicle id: %d, behavior: %d, x:%.1f, y:%.1f, d:%.1f",
                #                 lane_id, rear_vehicle.uid, rear_vehicle.behavior,
                #                 rear_vehicle.state.pose.pose.position.x,rear_vehicle.state.pose.pose.position.y,
                #                 rear_vehicle.ffstate.s)

    def locate_traffic_light_in_lanes(self, tstates):
        # TODO: Currently it's a very simple rule to locate the traffic lights
        if tstates.traffic_light_detection is None:
            total_lane_num = len(tstates.static_map.lanes)
            for i in range(total_lane_num):
                tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
            return
        
        lights = tstates.traffic_light_detection.detections
        total_lane_num = len(tstates.static_map.lanes)
        if len(lights) == 1:
            for i in range(total_lane_num):
                if lights[0].signal.flags == ObjectSignals.TRAFFIC_LIGHT_RED:
                    rospy.logwarn("RECEIVE RED LIGHT")
                    tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                elif lights[0].signal.flags == ObjectSignals.TRAFFIC_LIGHT_YELLOW:
                    tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_YIELD
                elif lights[0].signal.flags == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        
        # elif len(lights) > 1 and len(lights) == total_lane_num:
        #     for i in range(total_lane_num):
        #         if lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_RED:
        #             tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
        #         elif lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_YELLOW:
        #             tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_YIELD
        #         elif lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
        #             tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        # elif len(lights) > 1 and len(lights) != total_lane_num:
        #     red = True
        #     for i in range(len(lights)):
        #         if lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
        #             red = False
        #     for i in range(total_lane_num):
        #         if red:
        #             tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
        #         else:
        #             tstates.dynamic_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        
    def locate_stop_sign_in_lanes(self, tstates):
        '''
        Put stop sign detections into lanes
        '''

        # TODO: Implement this
        pass

    def locate_speed_limit_in_lanes(self, tstates):
        '''
        Put stop sign, traffic lights into lanes
        '''

        for i, lane in enumerate(tstates.dynamic_map.mmap.lanes):
            traffic_rule_speed_limit = tstates.static_map.lanes[i].speed_limit
            traffic_light_available_speed = self.traffic_light_available_speed(lane, tstates.static_map.lanes[i].traffic_light_pos)
            lane_tail_available_speed = self.lane_tail_available_speed(lane.ego_dis_to_lane_tail,0) #FIXME
            lane.map_lane.speed_limit = max(0, min(traffic_rule_speed_limit, 
                                            traffic_light_available_speed, 
                                            lane_tail_available_speed))
            rospy.logdebug("Cognition: lane id: %d, speed limit: %.1f km/h, rule:%.1f km/h, "
                           "traffic light:%.1f km/h, lane tail:%.1f km/h",
                           i, lane.map_lane.speed_limit, traffic_rule_speed_limit,
                           traffic_light_available_speed, lane_tail_available_speed)

    def traffic_light_available_speed(self, lane, traffic_light_pos):

        if lane.map_lane.stop_state == Lane.STOP_STATE_THRU:
            return float("inf")

        d = 0
        
        for pos in traffic_light_pos:
            if lane.ego_dis_to_lane_tail < pos:
                continue
            d = lane.ego_dis_to_lane_tail - pos
            break

        return self.lane_tail_available_speed(d, 0)

    def navigation_available_speed(self, tstates):

        return 0

    def lane_tail_available_speed(self, distance_to_tail, tail_speed=20/3.6, 
                                        d_thres=5, dec=0.2):
        '''
        Calculate the speed when close to the tail
        TODO(zhcao): should according to the tail required speed, if tail speed is 0, it is a stop sign or red traffic light
        '''
        if distance_to_tail <= d_thres:
            return tail_speed

        available_speed = math.sqrt(2*dec*distance_to_tail+tail_speed**2)  # m/s
        
        return available_speed*3.6
      
    def predict_vehicle_behavior(self, vehicle, tstates, lane_change_thres = 0.2):
        '''
        Detect the behaviors of surrounding vehicles
        '''

        dist_list = np.array([dist_from_point_to_polyline2d(vehicle.state.pose.pose.position.x, vehicle.state.pose.pose.position.y, lane)
            for lane in tstates.static_map_lane_path_array])
        dist_list = np.abs(dist_list)
        closest_lane = dist_list[:, 0].argsort()[0]
        closest_idx = int(dist_list[closest_lane, 1])
        closest_point = tstates.dynamic_map.mmap.lanes[closest_lane].map_lane.central_path_points[closest_idx]

        vehicle_driving_direction = get_yaw(vehicle.state)
        lane_direction = closest_point.tangent
        d_theta = vehicle_driving_direction - lane_direction
        d_theta = wrap_angle(d_theta)

        if abs(d_theta) > lane_change_thres:
            if d_theta > 0:
                behavior = RoadObstacle.BEHAVIOR_MOVING_LEFT
            else:
                behavior = RoadObstacle.BEHAVIOR_MOVING_RIGHT
        else:
            behavior = RoadObstacle.BEHAVIOR_FOLLOW
        
        return behavior
