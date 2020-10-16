
import rospy
import numpy as np
import math

from zzz_driver_msgs.utils import get_speed
from zzz_cognition_msgs.msg import MapState, RoadObstacle


class LaneUtility(object):

    def __init__(self, longitudinal_model):
        self.longitudinal_model_instance = longitudinal_model
        self.dynamic_map = None
        self.decision_action = 0.0

    def lateral_decision(self, dynamic_map):

        self.longitudinal_model_instance.update_dynamic_map(dynamic_map)
        self.dynamic_map = dynamic_map

        target_index = self.generate_lane_change_index()
        target_speed = self.longitudinal_model_instance.longitudinal_speed(target_index)

        return target_index, target_speed

    def generate_lane_change_index(self):

        candidate_lanes = self.get_candidate_lanes()
        candidate_lanes_utility = [self.lane_utility(int(lane_index)) for lane_index in candidate_lanes]
        target_lane = int(candidate_lanes[np.argmax(np.array(candidate_lanes_utility))])

        return target_lane

    def get_candidate_lanes(self, available_lane_change_range=1.2):
        candidate_lanes = np.arange(
            math.ceil(self.dynamic_map.mmap.ego_lane_index - available_lane_change_range),
            math.floor(self.dynamic_map.mmap.ego_lane_index + available_lane_change_range)
        )

        return candidate_lanes

    def lane_utility(self, lane_index):

        ego_lane_index = int(round(self.dynamic_map.mmap.ego_lane_index))

        if not self.lane_change_safe(ego_lane_index, lane_index):
            return -1

        available_speed = self.longitudinal_model_instance.longitudinal_speed(lane_index)
        exit_lanes = np.array(self.dynamic_map.mmap.exit_lane_index)
        mandatory_lanes_num = min(abs(exit_lanes - lane_index))
        distance_to_end = self.dynamic_map.mmap.distance_to_junction
        utility = available_speed*1.5 + 1/(mandatory_lanes_num+1)*max(0, (200-distance_to_end))*0.1

        if ego_lane_index == lane_index:
            utility += 0.5

        return utility

    def lane_change_safe(self, ego_lane_index, target_index):

        if ego_lane_index == target_index:
            return True

        if target_index < 0 or target_index > len(self.dynamic_map.mmap.lanes)-1:
            return False

        front_safe = False
        rear_safe = False
        front_vehicle = None
        rear_vehicle = None
        ego_vehicle_location = np.array([self.dynamic_map.ego_state.pose.pose.position.x,
                                         self.dynamic_map.ego_state.pose.pose.position.y])

        target_lane = self.dynamic_map.mmap.lanes[target_index]

        if len(target_lane.front_vehicles) > 0:
            front_vehicle = target_lane.front_vehicles[0]
        
        if len(target_lane.rear_vehicles) > 0:
            rear_vehicle = target_lane.rear_vehicles[0]

        ego_v = get_speed(self.dynamic_map.ego_state)

        if front_vehicle is None:
            d_front = -1
            front_safe = True
            behavior_front = RoadObstacle.BEHAVIOR_UNKNOWN
        else:
            front_vehicle_location = np.array([
                front_vehicle.state.pose.pose.position.x,
                front_vehicle.state.pose.pose.position.y
            ])
            behavior_front = front_vehicle.behavior
            # TODO: Change to real distance in lane
            d_front = np.linalg.norm(front_vehicle_location - ego_vehicle_location)
            front_v = get_speed(front_vehicle.state)
            if d_front > max(10 + 3*(ego_v-front_v), 20):
                front_safe = True

        if rear_vehicle is None:
            rear_safe = True
            d_rear = -1
            behavior_rear = RoadObstacle.BEHAVIOR_UNKNOWN
        else:
            rear_vehicle_location = np.array([
                rear_vehicle.state.pose.pose.position.x,
                rear_vehicle.state.pose.pose.position.y
            ])
            behavior_rear = rear_vehicle.behavior
            d_rear = np.linalg.norm(rear_vehicle_location - ego_vehicle_location)
            rear_v = get_speed(rear_vehicle.state)
            if d_rear > max(10 + 3*(rear_v-ego_v), 20):
                rear_safe = True

        # rospy.logdebug("ego_lane = %d, safe_checking_target_lane = %d, front_d = %f(%d), rear_d = %f(%d)",
        #                         ego_lane_index, target_index, d_front,
        #                         behavior_front, d_rear, behavior_rear)
                                
        if front_safe and rear_safe:
            return True
            
        return False

class MOBIL(object):
    pass
