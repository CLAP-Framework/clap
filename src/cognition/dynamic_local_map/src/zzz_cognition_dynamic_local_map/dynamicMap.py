import numpy as np
import numpy.linalg as npl
from copy import deepcopy
import rospy

from zzz_driver_msgs.msg import MapState
from zzz_library import dist_from_point_to_line

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
        self.ego_vehicle_lane_point_index = []
        self.surrounding_vehicle_list = []

        self.reference_path = deque(maxlen=20000)
        self.reference_path_buffer = deque(maxlen=200)
        self.trajectory = deque()
        self.judge_LC_thr = 3

        self.static_map_lane_path_array = None # list of point array of a lane

    def setup(self):
        pass

    def setup_reference_path(self,reference_path):
        self.reference_path.clear()
        for i, wp in enumerate(reference_path.poses):
            self.reference_path.append(np.array([wp.pose.position.x,-wp.pose.position.y]))

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

    """
    Main Update
    """
    def update_dynamic_map(self,ego_x,ego_y):

        self.ego_vehicle_location_x = ego_x
        self.ego_vehicle_location_y = ego_y
        self.ego_vehicle_location = np.array([ego_x,ego_y])
        self.dynamic_map = deepcopy(self.static_map)

        self.update_reference_path_buffer()

        if self.get_lane_count() == 0 or self.dynamic_map.in_junction:
            self.dynamic_map.in_junction = True
        if not self.dynamic_map.in_junction:
            self.update_ego_vehicle_index()
        if not self.dynamic_map.in_junction:
            self.update_surrounding_vehicle_in_lanes()

        rospy.logdebug("Updated Dynamic Map: lanes_num = %d, in_junction = %d, ego_y = %d",
                                                len(self.static_map.lanes),
                                                int(self.static_map.in_junction),
                                                self.dynamic_map.ego_lane_index)


    def convert_lane_to_ndarray(self):
        self.static_map_lane_path_array = []
        for lane in self.static_map.lanes:
            point_list = [(pose.pose.position.x, pose.pose.position.y) for pose in lane.central_path.poses]
            self.static_map_lane_path_array.append(np.array(point_list))
            # print(point_list)

    def get_lane_distances(self, x, y):
        '''
        Return (index of the closest point in each lane, distance to the closest point in each lane)
        '''
        idx_lane_closest_point = []
        dist_lane_closest_point = []

        for path in self.static_map_lane_path_array:
            dist_to_points = npl.norm(path - [x, y], axis=1)
            idx_cpoint = np.argmin(dist_to_points)
            
            dist_previous = dist_next = float('inf')
            if idx_cpoint != 0:
                dist_previous = dist_from_point_to_line(x, y,
                    path[idx_cpoint-1, 0], path[idx_cpoint-1, 1],
                    path[idx_cpoint, 0], path[idx_cpoint, 1])

            if idx_cpoint != len(path) - 1:
                dist_next = dist_from_point_to_line(x, y,
                    path[idx_cpoint, 0], path[idx_cpoint, 1],
                    path[idx_cpoint+1, 0], path[idx_cpoint+1, 1])

            idx_lane_closest_point.append(idx_cpoint)
            dist_lane_closest_point.append(min(dist_previous, dist_next, dist_to_points[idx_cpoint]))
        
        return idx_lane_closest_point, dist_lane_closest_point

    def get_lane_count(self):
        return len(self.static_map.lanes)


    """
    For in lane
    """


    def update_ego_vehicle_index(self):      

        idx_lane_closest_points, dist_lane_closest_point = self.get_lane_distances(self.ego_vehicle_location_x, self.ego_vehicle_location_y)
        idx_closet_path = np.argmin(dist_lane_closest_point) 
        idx_closet_point = idx_lane_closest_points[idx_closet_path]
        # print(idx_lane_closest_points,dist_lane_closest_point,idx_closet_path,idx_closet_point,self.ego_vehicle_location_x,self.ego_vehicle_location_y)
        self.ego_vehicle_lane_point_index = idx_lane_closest_points
        
        if len(self.static_map_lane_path_array[idx_closet_path]) - idx_closet_point <= 2:
            self.dynamic_map.in_junction = True
        else:
            self.dynamic_map.in_junction = False
            self.dynamic_map.ego_lane_index = self.static_map.lanes[idx_closet_path].index
            paths_to_end = self.static_map_lane_path_array[idx_closet_path][idx_closet_point:]
            self.dynamic_map.distance_to_next_lane = np.sum(np.linalg.norm(np.diff(paths_to_end,axis=0),axis=1))

    def update_surrounding_vehicle_in_lanes(self):
        # TODO: Fix distance to front / rear vehicle
        # TODO: Remove vehicle in opposite direction
        lane_front_vehicle_point_list = {}
        lane_rear_vehicle_point_list = {}
        vehicle_id_list = []
        for vehicle in self.surrounding_vehicle_list:
            idx_lane_closest_points, dist_lane_closest_point = self.get_lane_distances(vehicle.obstacle_pos_x, vehicle.obstacle_pos_y)
            idx_closet_path = np.argmin(dist_lane_closest_point)
            vehicle_id_list.append((idx_lane_closest_points[idx_closet_path], idx_closet_path))
        
        for lane_id in range(self.get_lane_count()):
            current_lane_vehicles = [(idx_vehicle, idx_point) for idx_vehicle, (idx_point, idx_path)
                in enumerate(vehicle_id_list) if idx_path == lane_id]
            if len(current_lane_vehicles) == 0: continue
            current_lane_vehicles = np.array(current_lane_vehicles)

            front_vehicles = current_lane_vehicles[current_lane_vehicles[:,1] > self.ego_vehicle_lane_point_index[lane_id]]
            rear_vehicles = current_lane_vehicles[current_lane_vehicles[:,1] < self.ego_vehicle_lane_point_index[lane_id]]
            if len(front_vehicles) > 0:
                self.static_map.lanes[lane_id].front_vehicle = current_lane_vehicles[np.argmin(front_vehicles, axis=0)[1], 0]
                rospy.logdebug("Lane index: %d, Front vechile id, %d", lane_id, self.static_map.lanes[lane_id].front_vehicle)                
            if len(rear_vehicles) > 0:
                self.static_map.lanes[lane_id].rear_vehicle  = current_lane_vehicles[np.argmax(rear_vehicles , axis=0)[1], 0]
                rospy.logdebug("Lane index: %d, Rear vechile id, %d", lane_id, self.static_map.lanes[lane_id].rear_vehicle) 

    
    """
    For reference path
    """

    def update_reference_path_buffer(self):

        # find the nearest point
        min_distance = 50
        min_index = 0
        index = 0

        while index < 40 and index < len(self.reference_path_buffer) and self.reference_path_buffer[index]:
            d = np.linalg.norm(self.reference_path_buffer[index]-self.ego_vehicle_location)
            if d < min_distance:
                min_distance = d
                min_index = index
            index += 1

        for index in range(0, min_index):
            self.reference_path_buffer.popleft()

        while self.reference_path and len(self.reference_path_buffer) < self._buffer_size:
            wp = self.reference_path.popleft()
            self.lane_change_smoothen(wp)
            self.reference_path_buffer.append(wp)
        
        Rpath = Path()
        for wp in self.reference_path_buffer:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            Rpath.poses.append(pose)
        
        front_vehicle = self.get_front_vehicle_on_reference_path()
        if front_vehicle is not None:
            self.dynamic_map.front_vehicle_on_reference_path = front_vehicle
            self.dynamic_map.have_front_vehicle_on_reference_path = True
        self.dynamic_map.reference_path_buffer = Rpath

    def lane_change_smoothen(self,wp):

        if self.reference_path_buffer:
            last_wp = self.reference_path_buffer[-1]
        else:
            return
        if np.linalg.norm(last_wp-wp) < self.judge_lc_thr:
            return

        ## Start smoothen the reference path
        rospy.logdebug("Reference Path Smoothing")
        lane_change_distance = min(20,len(self.reference_path_buffer)) # TODO
        last_wp = wp
        first_wp = self.reference_path_buffer[-lane_change_distance]
        loc_xs = np.linspace(last_wp[0], first_wp[0],num = lane_change_distance+1)
        loc_ys = np.linspace(last_wp[1], first_wp[1],num = lane_change_distance+1)
        for i in range(1,lane_change_distance):
            self.reference_path_buffer[-1] = np.array([loc_xs[i],loc_ys[i]])  

    def get_front_vehicle_on_reference_path(self):

        front_vehicle = None
        reference_path = np.array(reference_path)
        for vehicle in self.surrounding_vehicle_list:
            vehicle_loc = np.array([vehicle.obstacle_pos_x,vehicle.obstacle_pos_y])


            d = np.linalg.norm(vehicle_loc - self.ego_vehicle_location)

            if location_on_the_path(local_path,target_vehicle.location,gap_between_two_points+2.6):
                if d < nearest_distance:
                    nearest_distance = d
                    front_vehicle = target_vehicle

        if front_vehicle is not None:
            d = distance_between_two_loc(front_vehicle.location, EnvironmentInfo.ego_vehicle_location)
            print("front vehicle distance", d)
        else:
            print("front vehicle distance: None")
        
        return front_vehicle