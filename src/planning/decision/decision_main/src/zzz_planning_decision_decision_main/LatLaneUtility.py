
import rospy
import numpy as np


class LatLaneUtility(object):

    def __init__(self,longitudinal_model):
        self.longitudinal_model_instance = longitudinal_model
        self.dynamic_map = None

    def lateral_decision(self,dynamic_map,close_to_junction = 20):

        self.longitudinal_model_instance.update_dynamic_map(dynamic_map)
        self.dynamic_map = dynamic_map

        ### Following reference path
        if dynamic_map.in_junction or len(dynamic_map.lanes) < 2 or dynamic_map.target_lane_index == -1:
            return -1, self.longitudinal_model_instance.IDM_speed(-1)

        ### Cannot locate ego vehicle rightly
        if dynamic_map.ego_lane_index < 0 or dynamic_map.ego_lane_index > len(dynamic_map.lanes)-1:
            return -1, self.longitudinal_model_instance.IDM_speed(-1)

        if dynamic_map.distance_to_next_lane < close_to_junction:
            return -1, self.longitudinal_model_instance.IDM_speed(-1)

        target_index = self.generate_lane_change_index()
        # ego_lane = self.dynamic_map.lanes[0]

        # if ego_lane.have_front_vehicle:
        #     front_vehicle = ego_lane.front_vehicle
        #     front_vehicle_loc = np.array([front_vehicle.obstacle_pos_x,front_vehicle.obstacle_pos_y])
        #     ego_loc = np.array([dynamic_map.ego_vehicle_pose.position.x,dynamic_map.ego_vehicle_pose.position.y])
        #     d = np.linalg.norm(front_vehicle_loc-ego_loc)

        target_speed = self.longitudinal_model_instance.IDM_speed(dynamic_map.ego_lane_index,traffic_light = True)
        # TODO: More accurate speed
        
        return target_index, target_speed

    def generate_lane_change_index(self):

        ego_lane_index = self.dynamic_map.ego_lane_index
        current_lane_utility = self.lane_utility(ego_lane_index)

        if not self.lane_change_safe(ego_lane_index,ego_lane_index + 1):
            left_lane_utility = -1
        else:
            left_lane_utility = self.lane_utility(self.dynamic_map.ego_lane_index + 1)

        if not self.lane_change_safe(ego_lane_index,ego_lane_index - 1):
            right_lane_utility = -1
        else:
            right_lane_utility = self.lane_utility(self.dynamic_map.ego_lane_index - 1)

        # TODO: target lane = -1?
        rospy.logdebug("left_utility = %f, ego_utility = %f, right_utility = %f",
                                                                left_lane_utility,
                                                                current_lane_utility,
                                                                right_lane_utility)


        if right_lane_utility > current_lane_utility and right_lane_utility >= left_lane_utility:
            return self.dynamic_map.ego_lane_index -1

        if left_lane_utility > current_lane_utility and left_lane_utility > right_lane_utility:
            return self.dynamic_map.ego_lane_index + 1

        return self.dynamic_map.ego_lane_index

        

    def lane_utility(self,lane_index):

        available_speed = self.longitudinal_model_instance.IDM_speed(lane_index)
        exit_lane_index = self.dynamic_map.target_lane_index
        distance_to_end = self.dynamic_map.distance_to_next_lane
        utility = available_speed + 1/(abs(exit_lane_index - lane_index)+1)*max(0,(260-distance_to_end))

        return utility


    def lane_change_safe(self,ego_lane_index,target_index):

        if target_index < 0 or target_index > len(self.dynamic_map.lanes)-1:
            return False

        front_safe = False
        rear_safe = False
        front_vehicle = None
        rear_vehicle = None
        ego_vehicle_location = np.array([self.dynamic_map.ego_vehicle_pose.position.x,
                                         self.dynamic_map.ego_vehicle_pose.position.y])

        for lane in self.dynamic_map.lanes:
            if lane.index == target_index:
                if lane.have_front_vehicle:
                    front_vehicle = lane.front_vehicle
                if lane.have_rear_vehicle:
                    rear_vehicle = lane.rear_vehicle
                break

        if front_vehicle is None:
            front_safe = True
        else:
            front_vehicle_location = np.array([front_vehicle.obstacle_pos_x,front_vehicle.obstacle_pos_y])
            d_front = np.linalg.norm(front_vehicle_location-ego_vehicle_location)
            front_v = front_vehicle.obstacle_speed
            ego_v = self.dynamic_map.ego_vehicle_speed
            if d_front > max((10 + 3*(ego_v-front_v)),10):
                front_safe = True
        

        if rear_vehicle is None:
            rear_safe = True

        else:
            rear_vehicle_location = np.array([rear_vehicle.obstacle_pos_x,rear_vehicle.obstacle_pos_y])
            d_rear = np.linalg.norm(rear_vehicle_location-ego_vehicle_location)
            rear_v = lane.rear_vehicle.obstacle_speed
            ego_v = self.dynamic_map.ego_vehicle_speed

            if d_rear > max((10 + 3*(rear_v-ego_v)),10):
                rear_safe = True

        if front_safe and rear_safe:
            return True
            
        return False
 
