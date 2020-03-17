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
    def __init__(self, trajectory_planner=None):
        self._dynamic_map_buffer = None

        self._trajectory_planner = trajectory_planner
        self.solve = False

        self.Planning_type = 3

        
    def receive_dynamic_map(self, dynamic_map):
        self._dynamic_map_buffer = dynamic_map

    def update_trajectory(self):
        
        # This function generate trajectory

        # Update_dynamic_local_map
        if self._dynamic_map_buffer is None:
            return None
        dynamic_map = self._dynamic_map_buffer
        reference_path_from_map = dynamic_map.jmap.reference_path

        if self.Planning_type == 1:
            # Only follow path
            trajectory = reference_path_from_map
            desired_speed = self._trajectory_planner.speed_update(trajectory, dynamic_map)

            # Process reference path
            if trajectory is not None:
                send_trajectory = self.process_reference_path(trajectory,dynamic_map,desired_speed)

            # Convert to Message type
            if send_trajectory is not None:
                msg = DecisionTrajectory()
                msg.trajectory = self.convert_ndarray_to_pathmsg(send_trajectory) # TODO: move to library
                msg.desired_speed = desired_speed 

            return msg


        elif self.Planning_type == 2:
            # Keep replanning
            
            # Should use lane model
            if dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP:
                if self.solve == False:
                    self._trajectory_planner.something_in_lane(dynamic_map)
                    self.solve = True
                return None
            elif dynamic_map.model == dynamic_map.MODEL_JUNCTION_MAP:
                self.solve = False
            try:
                trajectory, desired_speed = self._trajectory_planner.trajectory_update(dynamic_map)
                if trajectory is not None:
                    msg = DecisionTrajectory()
                    msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory)
                    msg.desired_speed = desired_speed
                    return msg
                else:
                    return None
            except:
                return None

        elif self.Planning_type == 3:
            # Keep replanning
            self._trajectory_planner.clear_buff(dynamic_map)
            if dynamic_map.model == dynamic_map.MODEL_JUNCTION_MAP:
                print("dynamic map tell me its junction")
                trajectory_msg = self._trajectory_planner.trajectory_update(dynamic_map)
                return trajectory_msg
            elif dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP:
                return None

            


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

    def found_closest_obstacles(self, num, dynamic_map):
        closest_obs = []
        obs_tuples = []

        reference_path_from_map = self._dynamic_map.jmap.reference_path.map_lane.central_path_points
        ref_path_ori = self.convert_path_to_ndarray(reference_path_from_map)
        if ref_path_ori is not None:
            ref_path = dense_polyline2d(ref_path_ori, 1)
            ref_path_tangets = np.zeros(len(ref_path))
        else:
            return None
        
        for obs in self._dynamic_map.jmap.obstacles: 
            # calculate distance
            p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x , self._dynamic_map.ego_state.pose.pose.position.y])
            p2 = np.array([obs.state.pose.pose.position.x , obs.state.pose.pose.position.y])
            p3 = p2 - p1
            p4 = math.hypot(p3[0],p3[1])

            # transfer to frenet
            obs_ffstate = get_frenet_state(obs.state, ref_path, ref_path_tangets)
            one_obs = (obs.state.pose.pose.position.x , obs.state.pose.pose.position.y , obs.state.twist.twist.linear.x , obs.state.twist.twist.linear.y , p4 , obs_ffstate.s , -obs_ffstate.d , obs_ffstate.vs , obs_ffstate.vd)
            # if obs.ffstate.s > 0.01:
            obs_tuples.append(one_obs)
        
        sorted_obs = sorted(obs_tuples, key=lambda obs: obs[4])   # sort by distance
        i = 0
        for obs in sorted_obs:
            if i < num:
                closest_obs.append(obs)
                i = i + 1
            else:
                break
        
        return closest_obs
