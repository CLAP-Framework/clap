import numpy as np
import numpy.linalg as npl
from copy import deepcopy
import rospy
from collections import deque
from zzz_driver_msgs.msg import MapState
from zzz_common.geometry import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DynamicMap(object):


    """
    Init and Setup
    """

    def __init__(self):
        self.static_map = MapState()
        self.static_map.in_junction = True
        self.dynamic_map = None
        self.ego_vehicle_location_x = 0.0
        self.ego_vehicle_location_y = 0.0
        self.ego_vehicle_location = np.array([0,0])
        self.ego_vehicle_speed = 0.0
        self.ego_vehicle_distance_to_lane_head = 0 # distance from vehicle to lane start
        self.ego_vehicle_distance_to_lane_tail = 0 # distance from vehicle to lane end
        self.surrounding_vehicle_list = []

        self.reference_path = deque(maxlen=20000)
        self.reference_path_buffer = deque(maxlen=200)
        self._buffer_size = 200
        self.trajectory = deque()
        self.judge_LC_thr = 3

        self.traffic_light_info = None

        self.static_map_lane_path_array = None # list of point array of a lane

    def setup(self):
        pass

    def setup_reference_path(self,reference_path):
        self.reference_path.clear()
        for i, wp in enumerate(reference_path.poses):
            self.reference_path.append(np.array([wp.pose.position.x,-wp.pose.position.y]))

    def convert_lane_to_ndarray(self):
        self.static_map_lane_path_array = []
        for lane in self.static_map.lanes:
            point_list = [(pose.pose.position.x, pose.pose.position.y) for pose in lane.central_path.poses]
            self.static_map_lane_path_array.append(np.array(point_list))

    def update_static_map(self,static_map):
        self.static_map = static_map
        self.convert_lane_to_ndarray()
        rospy.loginfo("Updated Local Static Map: lanes_num = %d, in_junction = %d, target_lane_index = %d",
                                                            len(self.static_map.lanes), 
                                                            int(self.static_map.in_junction),
                                                            self.static_map.target_lane_index)

    def update_vehicle_list(self,vehicle_list):
        # TODO: data.obstcles
        self.surrounding_vehicle_list = vehicle_list

    def update_ego_speed(self,speed):
        self.ego_vehicle_speed = speed

    def update_traffic_light_info(self, traffic_light_info):
        self.traffic_light_info = traffic_light_info

    """
    Main Update
    """
    def update_dynamic_map(self,ego_pose):

        self.ego_vehicle_location_x = ego_pose.position.x
        self.ego_vehicle_location_y = ego_pose.position.y
        self.ego_vehicle_location = np.array([self.ego_vehicle_location_x,self.ego_vehicle_location_y])

        self.dynamic_map = deepcopy(self.static_map)
        self.dynamic_map.ego_vehicle_pose = ego_pose
        self.dynamic_map.ego_vehicle_speed = self.ego_vehicle_speed

        self.update_reference_path_buffer()
        self.update_traffic_light()
        self.update_stop_sign()

        if self.get_lane_count() == 0:
            self.dynamic_map.in_junction = True
        if not self.dynamic_map.in_junction:
            self.update_ego_vehicle_index()
            self.update_surrounding_vehicle_in_lanes()
        self.update_traffic_light_in_lane()
        
        self.update_speed_limit()

        rospy.logdebug("Updated Dynamic Map: lanes_num = %d, in_junction = %d, ego_y = %d, distance_to_end = %f",
                                                len(self.static_map.lanes),
                                                int(self.static_map.in_junction),
                                                self.dynamic_map.ego_lane_index,
                                                self.dynamic_map.distance_to_next_lane)

    def get_lane_count(self):
        return len(self.static_map.lanes)

    """
    For in lane
    """

    def update_ego_vehicle_index(self, lane_end_dist_thres=2, lane_dist_thres=5): 
        dist_list = np.array([dist_from_point_to_polyline(self.ego_vehicle_location_x, self.ego_vehicle_location_y, lane)
            for lane in self.static_map_lane_path_array])  
        closest_lane = np.argmin(dist_list[:, 0])

        if dist_list[closest_lane, 0] > lane_dist_thres:
            self.dynamic_map.in_junction = True
            return

        self.ego_vehicle_distance_to_lane_head = dist_list[closest_lane, 1]
        self.ego_vehicle_distance_to_lane_tail = dist_list[closest_lane, 2]
        
        if self.ego_vehicle_distance_to_lane_tail <= lane_end_dist_thres:
            # At the end of the lane
            self.dynamic_map.in_junction = True
            ## drive into junction, wait until next map # TODO: change the condition
            self.static_map.in_junction = True
        else:
            self.dynamic_map.in_junction = False
            self.dynamic_map.ego_lane_index = self.static_map.lanes[closest_lane].index
            self.dynamic_map.distance_to_next_lane = self.ego_vehicle_distance_to_lane_tail

    def update_surrounding_vehicle_in_lanes(self, lane_dist_thres=2):
        lane_front_vehicle_list = [[] for _ in self.static_map.lanes]
        lane_rear_vehicle_list = [[] for _ in self.static_map.lanes]

        for vehicle_idx, vehicle in enumerate(self.surrounding_vehicle_list):
            dist_list = np.array([dist_from_point_to_polyline(vehicle.obstacle_pos_x, vehicle.obstacle_pos_y, lane)
                for lane in self.static_map_lane_path_array])
            dist_list = np.abs(dist_list)
            closest_lane = np.argmin(dist_list[:, 0])

            # Determine if the vehicle is close to lane enough
            if dist_list[closest_lane, 0] > lane_dist_thres:
                continue 
            if dist_list[closest_lane, 1] < self.ego_vehicle_distance_to_lane_head:
                lane_rear_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane,1]))
            if dist_list[closest_lane, 2] < self.ego_vehicle_distance_to_lane_tail:
                lane_front_vehicle_list[closest_lane].append((vehicle_idx, dist_list[closest_lane,2]))
        
        for lane_id in range(self.get_lane_count()):
            front_vehicles = np.array(lane_front_vehicle_list[lane_id])
            rear_vehicles = np.array(lane_rear_vehicle_list[lane_id])
            if len(front_vehicles) > 0:
                front_vehicle_idx = int(front_vehicles[np.argmin(front_vehicles[:,1]), 0])
                self.static_map.lanes[lane_id].front_vehicle = self.surrounding_vehicle_list[front_vehicle_idx]
                rospy.logdebug("Lane index: %d, Front vehicle id: %d", lane_id, self.static_map.lanes[lane_id].front_vehicle.obstacle_id)                
            if len(rear_vehicles) > 0:
                rear_vehicle_idx  = int(rear_vehicles [np.argmin(rear_vehicles[:,1]), 0])
                self.static_map.lanes[lane_id].rear_vehicle  = self.surrounding_vehicle_list[rear_vehicle_idx]
                rospy.logdebug("Lane index: %d, Rear vehicle id: %d", lane_id, self.static_map.lanes[lane_id].rear_vehicle.obstacle_id) 

    def update_traffic_light_in_lane(self):
        if self.traffic_light_info is None:
            return
        lights = self.traffic_light_info.detections

        total_lane_num = len(self.dynamic_map.lanes)
        if len(lights) == 1:
            for i in range(total_lane_num):
                if lights[0].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_RED:
                    self.dynamic_map.lanes[i].traffic_light_state = 1
                elif lights[0].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_YELLOW:
                    self.dynamic_map.lanes[i].traffic_light_state = 2
                elif lights[0].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_GREEN:
                    self.dynamic_map.lanes[i].traffic_light_state = 3
        elif len(lights) > 1 and len(lights) == total_lane_num:
            for i in range(total_lane_num):
                if lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_RED:
                    self.dynamic_map.lanes[i].traffic_light_state = 1
                elif lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_YELLOW:
                    self.dynamic_map.lanes[i].traffic_light_state = 2
                elif lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_GREEN:
                    self.dynamic_map.lanes[i].traffic_light_state = 3
        elif len(lights) > 1 and len(lights) != total_lane_num:
            red = True
            for i in range(len(lights)):
                if lights[i].traffic_light_state == TrafficLightDetection.TRAFFIC_LIGHT_GREEN:
                    red = False
            for i in range(total_lane_num):
                if red:
                    self.dynamic_map.lanes[i].traffic_light_state = 1
                else:
                    self.dynamic_map.lanes[i].traffic_light_state = 3
        

    """
    For reference path
    """

    def update_reference_path_buffer(self):
        """
        Delete the passed point and add more point to the reference path
        """
 
        min_distance = 50
        min_index = 0
        index = 0

        if self.reference_path_buffer:
            nearest_dis, nearest_idx = nearest_point_to_polyline(self.ego_vehicle_location[0],
                                                                 self.ego_vehicle_location[1],
                                                                 np.array(self.reference_path_buffer))
            for index in range(0, nearest_idx):
                self.reference_path_buffer.popleft()


        while self.reference_path and len(self.reference_path_buffer) < self._buffer_size:
            wp = self.reference_path.popleft()
            self.lane_change_smoothen(wp)
            self.reference_path_buffer.append(wp)

        # TODO: find front vehicle?
        

        Rpath = Path()
        for wp in self.reference_path_buffer:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            Rpath.poses.append(pose)
        
        self.dynamic_map.reference_path.central_path = Rpath
        self.dynamic_map.reference_path.index = -1

        front_vehicle = self.get_front_vehicle_on_reference_path()
        if front_vehicle is None:
            self.dynamic_map.reference_path.have_front_vehicle = False
        else:
            self.dynamic_map.reference_path.have_front_vehicle = True
            self.dynamic_map.reference_path.front_vehicle = front_vehicle

        self.dynamic_map.reference_path.speed_limit = 30

        

    def lane_change_smoothen(self,wp):
        """
        Avoid suddenly lane change
        """
        if self.reference_path_buffer:
            last_wp = self.reference_path_buffer[-1]
        else:
            return
        if np.linalg.norm(last_wp-wp) < self.judge_LC_thr:
            return

        ## Start smoothen the reference path
        rospy.logdebug("Reference Path Smoothing")
        lane_change_distance = min(20,len(self.reference_path_buffer)) # TODO: Dynamic LC distance
        last_wp = wp
        first_wp = self.reference_path_buffer[-lane_change_distance]
        loc_xs = np.linspace(last_wp[0], first_wp[0],num = lane_change_distance+1)
        loc_ys = np.linspace(last_wp[1], first_wp[1],num = lane_change_distance+1)
        for i in range(1,lane_change_distance):
            self.reference_path_buffer[-1] = np.array([loc_xs[i],loc_ys[i]])  

    def get_front_vehicle_on_reference_path(self, lane_dist_thres=2):
        """
        Get front vehicle on the reference path
        """

        front_vehicle = None
        nearest_dis = 200
        reference_path = np.array(self.reference_path_buffer)

        for vehicle in self.surrounding_vehicle_list:

            dist_list = np.linalg.norm(reference_path - [vehicle.obstacle_pos_x,vehicle.obstacle_pos_y],axis = 1)

            min_dis = np.min(dist_list)
            min_idx = np.argmin(dist_list)

            if min_dis > lane_dist_thres:
                continue

            d = np.linalg.norm([vehicle.obstacle_pos_x,vehicle.obstacle_pos_y]-self.ego_vehicle_location)

            if d < nearest_dis:
                front_vehicle = vehicle
                nearest_dis = d

        if front_vehicle is not None:
            rospy.logdebug("reference lane: front vehicle dis: %f", nearest_dis)

        return front_vehicle

    """
    Traffic light
    """
    def update_traffic_light(self):
        
        pass


    """
    Stop sign
    """
    def update_stop_sign(self):

        pass

    """
    Speed limit
    """
    def update_speed_limit(self):
        pass