#!/usr/bin/env python

import rospy
import numpy as np
from zzz_planning_decision_lane_models.longitudinal import IDM
from zzz_planning_decision_lane_models.lateral import LaneUtility
from zzz_common.geometry import dense_polyline, nearest_point_to_polyline
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Make lat lon model as parameter 

class MainDecision(object):
    def __init__(self):
        self.dynamic_map = None
        self.longitudinal_model_instance = IDM()
        self.lateral_model_instance = LaneUtility(self.longitudinal_model_instance)

    # Loop entry
    def generate_trajectory_with_speed(self, dynamic_map):

        # update_dynamic_local_map
        self.dynamic_map = dynamic_map

        changing_lane_index, desired_speed = self.lateral_model_instance.lateral_decision(dynamic_map)
        if desired_speed < 0:
            desired_speed = 0
        rospy.logdebug("target_lane_index = %d, target_speed = %f km/h", changing_lane_index, desired_speed*3.6)

        # TODO: Is this reasonable?
        # if len(self.dynamic_map.jmap.reference_path.map_lane.central_path_points) == 0:
        #     return DecisionTrajectory() # Return null trajectory

        # get trajectory by target lane and desired speed
        trajectory = self.get_trajectory(changing_lane_index, desired_speed)

        msg = DecisionTrajectory()
        msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory) # TODO: move to library
        msg.desired_speed = desired_speed

        return msg

    def get_trajectory(self, changing_lane_index, desired_speed, resolution=0.5, time_ahead=5, distance_ahead=10, rectify_thres=4):
        # TODO: get smooth spline (write another module to generate spline)
        ego_x = self.dynamic_map.ego_state.pose.pose.position.x
        ego_y = self.dynamic_map.ego_state.pose.pose.position.y
        lane = self.get_lane_by_index(changing_lane_index)
        central_path = self.convert_path_to_ndarray(lane.map_lane.central_path_points)
        
        dense_centrol_path = dense_polyline(central_path, resolution)
        nearest_dis, nearest_idx = nearest_point_to_polyline(ego_x, ego_y, dense_centrol_path)
        
        front_path = dense_centrol_path[nearest_idx:]
        # correct if ego vehicle is away from target trajectory
        # if nearest_dis > rectify_thres:

        dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))
        trajectory = front_path[:np.searchsorted(dis_to_ego, desired_speed*time_ahead+distance_ahead)-1]
        return trajectory
    
    def get_lane_by_index(self,lane_index):

        if lane_index == -1:
            return self.dynamic_map.jmap.reference_path

        for lane in self.dynamic_map.mmap.lanes:
            if lane.map_lane.index == lane_index:
                return lane

        return None

    # TODO(zyxin): Add these to zzz_navigation_msgs.utils
    def convert_path_to_ndarray(self, path):
        point_list = [(point.position.x, point.position.y) for point in path]
        return np.array(point_list)

    def convert_ndarray_to_pathmsg(self, path):
        msg = Path()
        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            msg.poses.append(pose)

        return msg


