#!/usr/bin/env python

import rospy
import numpy as np
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed, get_yaw



class MainDecision(object):
    def __init__(self, path_decision=None, speed_decision=None, Planning_type=None):
        self._dynamic_map_buffer = None

        self._path_model_instance = path_decision
        self._speed_model_instance = speed_decision
        self.Planning_type = Planning_type
        
    def receive_dynamic_map(self, dynamic_map):
        self._dynamic_map_buffer = dynamic_map

    def update_trajectory(self):
        
        # This function generate trajectory

        # Update_dynamic_local_map
        if self._dynamic_map_buffer is None:
            return None
        dynamic_map = self._dynamic_map_buffer
        reference_path_from_map = dynamic_map.jmap.reference_path

        trajectory = reference_path_from_map
        desired_speed = 0

        if self.Planning_type == 1:
            # Only follow path
            trajectory = reference_path_from_map
            desired_speed = self._speed_model_instance.speed_update(trajectory, dynamic_map)

            # Process reference path  What happened here?? 
            if trajectory is not None:
                send_trajectory = self.process_reference_path(trajectory,dynamic_map,desired_speed)

            # Convert to Message type
            if send_trajectory is not None:
                msg = DecisionTrajectory()
                msg.trajectory = self.convert_ndarray_to_pathmsg(send_trajectory) # TODO: move to library
                msg.desired_speed = desired_speed  #FIXME(NANSHAN)
            return msg


        elif self.Planning_type == 2:
            # Keep replanning
            trajectory, desired_speed = self._path_model_instance.trajectory_update(dynamic_map)
            msg = DecisionTrajectory()
            msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory)
            msg.desired_speed = desired_speed

            return msg

        elif self.Planning_type == 3:
            # Replanning only when dynamic obstacles predict to collide with me

            # Collision check
            Collision_and_Replan = collision_check_dynamic(dynamic_map)
            if Collision_and_Replan == 1:
                trajectory = self._path_model_instance.trajectory_update(dynamic_map)

        elif self.Planning_type == 4:
            # Replanning only when fixed obstacles on my path

            # Collision check
            Collision_and_Replan = collision_check_fixed(dynamic_map)

            if Collision_and_Replan == 1:
                # Path update
                trajectory = self._path_model_instance.path_update(dynamic_map)

            # Speed update
            self._speed_model_instance.speed_update(trajectory, dynamic_map)


    def convert_XY_to_pathmsg(self,XX,YY,path_id = 'map'):
        msg = Path()
        print("....................Finish,Werling",XX,YY)

        for i in range(1,len(XX)):
            pose = PoseStamped()
            pose.pose.position.x = XX[i]
            pose.pose.position.y = YY[i]
            # print('trajectoryx=',XX[i])
            # print('trajectoryy=',YY[i])
            msg.poses.append(pose)
        msg.header.frame_id = path_id 
        return msg

    def process_reference_path(self,trajectory,dynamic_map,desired_speed):
        resolution=0.5
        time_ahead=5
        distance_ahead=10
        rectify_thres=2
        lc_dt = 1.5
        lc_v = 2.67
        ego_x = dynamic_map.ego_state.pose.pose.position.x
        ego_y = dynamic_map.ego_state.pose.pose.position.y
        central_path = self.convert_path_to_ndarray(trajectory.map_lane.central_path_points)
        dense_centrol_path = dense_polyline2d(central_path, resolution)
        nearest_dis, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_centrol_path)
        nearest_dis = abs(nearest_dis)
        front_path = dense_centrol_path[nearest_idx:]
        dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))
        return front_path[:np.searchsorted(dis_to_ego, desired_speed*time_ahead+distance_ahead)-1]


    def collision_check_fixed(self, dynamic_map):
        return 0

    def collision_check_dynamic(self, dynamic_map):
        return 0 


    def get_trajectory(self, dynamic_map, target_lane_index, desired_speed,
                resolution=0.5, time_ahead=5, distance_ahead=10, rectify_thres=2,
                lc_dt = 1.5, lc_v = 2.67):
        # TODO: get smooth spline (write another module to generate spline)
        ego_x = dynamic_map.ego_state.pose.pose.position.x
        ego_y = dynamic_map.ego_state.pose.pose.position.y
        if target_lane_index == -1:
            target_lane = dynamic_map.jmap.reference_path
        else:
            target_lane = dynamic_map.mmap.lanes[int(target_lane_index)]

        central_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)

        # if ego vehicle is on the target path
        # works for lane change, lane follow and reference path follow
        dense_centrol_path = dense_polyline2d(central_path, resolution)
        nearest_dis, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_centrol_path)
        nearest_dis = abs(nearest_dis)

        if nearest_dis > rectify_thres:
            if dynamic_map.model == MapState.MODEL_MULTILANE_MAP and target_lane_index != -1:
                rectify_dt = abs(dynamic_map.mmap.ego_lane_index - target_lane_index)*lc_dt
            else:
                rectify_dt = nearest_dis/lc_v
            return self.generate_smoothen_lane_change_trajectory(dynamic_map, target_lane, rectify_dt, desired_speed)

        else:
            front_path = dense_centrol_path[nearest_idx:]
            dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))
            trajectory = front_path[:np.searchsorted(dis_to_ego, desired_speed*time_ahead+distance_ahead)-1]
            return trajectory

    

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
        msg.header.frame_id = "map" 

        return msg

    def generate_smoothen_lane_change_trajectory(self, dynamic_map, target_lane,
        rectify_dt, desired_speed, lc_dt = 1.5, rectify_min_d = 6, resolution=0.5, time_ahead=5, distance_ahead=10):

        target_lane_center_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)

        ego_x = dynamic_map.ego_state.pose.pose.position.x
        ego_y = dynamic_map.ego_state.pose.pose.position.y

        # Calculate the longitudinal distance for lane Change
        # Considering if the ego_vehicle is in a lane Change
        lc_dis = max(rectify_dt*desired_speed,rectify_min_d)

        dense_target_centrol_path = dense_polyline2d(target_lane_center_path, resolution)
        _, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_target_centrol_path)
        front_path = dense_target_centrol_path[nearest_idx:]
        dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))

        start_point = np.array([ego_x,ego_y])
        end_point = front_path[np.searchsorted(dis_to_ego, lc_dis)]

        # calculate start direction and end direction for control

        ego_direction = get_yaw(dynamic_map.ego_state)
        _, nearest_end_idx, _ = dist_from_point_to_polyline2d(end_point[0], end_point[1], target_lane_center_path)
        end_point_direction = target_lane.map_lane.central_path_points[nearest_end_idx].tangent

        start_tangent = np.array([np.cos(ego_direction),np.sin(ego_direction)])
        end_tangent = np.array([np.cos(end_point_direction),np.sin(end_point_direction)])

        lc_path = self.cubic_hermite_spline(p0 = start_point, p1 = end_point,
                                            m0 = start_tangent, m1 = end_tangent)

        # get ahead Path
        ahead_dis = desired_speed*time_ahead+distance_ahead
        path_after_lc = front_path[np.searchsorted(dis_to_ego, lc_dis):np.searchsorted(dis_to_ego, ahead_dis)-1]

        # replace lane change path into ahead path
        smoothen_lc_path = np.concatenate((lc_path,path_after_lc),axis = 0)

        return smoothen_lc_path

    def cubic_hermite_spline(self, p0, p1, m0, m1, resolution = 20):
        """
        Generate cubic hermit spline
        p0: start point np.array(2)
        p1: end point np.array(2)
        m0: start tangent np.array(2)
        m1: end tangent np.array(2)
        return path from start point to end point
        """

        t = np.linspace(0,1,num = resolution)
        h00 = (2*t*t*t-3*t*t+1).reshape(len(t),1) #p0
        h10 = (t*t*t-2*t*t+t).reshape(len(t),1) #m0
        h01 = (-2*t*t*t+3*t*t).reshape(len(t),1) #p1
        h11 = (t*t*t-t*t).reshape(len(t),1) #m1

        p0 = p0.reshape(1,2)
        p1 = p1.reshape(1,2)
        m0 = m0.reshape(1,2)
        m1 = m1.reshape(1,2)

        return np.matmul(h00,p0) + np.matmul(h10,m0) + np.matmul(h01,p1) + np.matmul(h11,m1)
