

import numpy as np


class LonIDM(object):

    def __init__(self):

        self.T = 1.6
        self.g0 = 7
        self.a = 2.73
        self.b = 1.65
        self.delta = 4
        self.decision_dt = 0.75
        self.dynamic_map = None
    
    def update_dynamic_map(self,dynamic_map):
        self.dynamic_map = dynamic_map

    def IDM_speed(self,target_lane_index,traffic_light = False):

        target_lane = None

        if target_lane_index == -1:
            target_lane = self.dynamic_map.reference_path

        for lane in self.dynamic_map.lanes:
            if lane.index == target_lane_index:
                target_lane = lane

        if target_lane is None:
            return 0
        else:
            idm_speed = self.IDM_speed_in_lane(target_lane)
            traffic_light_speed = float("inf")
            if traffic_light:
                traffic_light_speed = self.traffic_light_speed(target_lane)
            return min(idm_speed,traffic_light_speed)

    def IDM_speed_in_lane(self,lane):

        ego_vehicle_location = np.array([self.dynamic_map.ego_vehicle_pose.position.x,
                                            self.dynamic_map.ego_vehicle_pose.position.y])

        v = self.dynamic_map.ego_vehicle_speed
        v0 = 30/3.6 #lane.speed_limit/3.6 TODO
        if v<5:
            a = self.a + (5-v)/5*2
        else:
            a = self.a
        
        b = self.b
        g0 = self.g0
        T = self.T
        delta = self.delta

        if lane.have_front_vehicle:
            f_v_location = np.array([lane.front_vehicle.obstacle_pos_x,lane.front_vehicle.obstacle_pos_y])
            v_f = lane.front_vehicle.obstacle_speed
            dv = v-v_f
            g = np.linalg.norm(f_v_location-ego_vehicle_location)
            g1 = g0+T*v+v*dv/(2*np.sqrt(a*b))
        else:
            dv = 0
            g = 50
            g1 = 0

        acc = a*(1-pow(v/v0,delta)-(g1/g)*((g1/g)))

        return v+acc*self.decision_dt

    
    def traffic_light_speed(self,lane):

        ego_vehicle_speed = self.dynamic_map.ego_vehicle_speed
        if lane.traffic_light_state != 1:
            return float("inf")
        else:
            d = lane.distance_to_traffic_light_line
            if d < 10 + ego_vehicle_speed*ego_vehicle_speed/2/2:
                return 0 

        return float("inf")
