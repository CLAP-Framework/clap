#!/usr/bin/env python

import rospy
import numpy as np
from LonIDM import LonIDM
from LatLaneUtility import LatLaneUtility
from zzz_common.geometry import *
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class MainDecision(object):
    def __init__(self):

        self.dynamic_map = None
        self.longitudinal_model_instance = LonIDM()
        self.lateral_model_instance = LatLaneUtility(self.longitudinal_model_instance)


    def generate_trajectory_with_speed(self,dynamic_map):

        # update_dynamic_local_map
        self.dynamic_map = dynamic_map
        

        changing_lane_index, desired_speed = self.lateral_model_instance.lateral_decision(dynamic_map)

        # changing_lane_index = -1
        # desired_speed = 30/3.6

        rospy.logdebug("target_lane_index = %d, target_speed = %f km/h", changing_lane_index, desired_speed*3.6)

        # get trajectory by target lane and desired speed
        trajectory = self.get_trajectory(changing_lane_index,desired_speed)

        msg = DecisionTrajectory()
        msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory) # TODO: move to library
        msg.desired_speed = desired_speed

        return msg


    def get_trajectory(self,changing_lane_index,desired_speed,resolution=0.5, time_ahead=5, distance_ahead=10, rectify_thres = 4):

        ego_x = self.dynamic_map.ego_vehicle_pose.position.x
        ego_y = self.dynamic_map.ego_vehicle_pose.position.y
        ego_loc = np.array([ego_x,ego_y])
        lane = self.get_lane_by_index(changing_lane_index)
        central_path = self.convert_path_to_ndarray(lane.central_path.poses)
        
        
        dense_centrol_path = dense_polyline(central_path,resolution)
        nearest_dis, nearest_idx = nearest_point_to_polyline(ego_x,ego_y,dense_centrol_path)
        
        
        front_path = dense_centrol_path[nearest_idx:]
        # correct if ego vehicle is away from target trajectory
        # if nearest_dis > rectify_thres:
            

        dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path,axis=0),axis = 1))
        trajectory = front_path[:np.searchsorted(dis_to_ego, desired_speed*time_ahead+distance_ahead)-1]
        return trajectory
    
    def get_lane_by_index(self,lane_index):

        if lane_index == -1:
            return self.dynamic_map.reference_path

        for lane in self.dynamic_map.lanes:
            if lane.index == lane_index:
                return lane

        return None

    def convert_path_to_ndarray(self,path):

        point_list = [(pose.pose.position.x, pose.pose.position.y) for pose in path]
        return np.array(point_list)

    def convert_ndarray_to_pathmsg(self,path):
        msg = Path()
        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            msg.poses.append(pose)

        return msg


