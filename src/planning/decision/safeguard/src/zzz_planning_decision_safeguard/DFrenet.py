#!/usr/bin/env python

import rospy
import numpy as np
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed, get_yaw
import math
from zzz_perception_msgs.msg import ObjectClass
from intervaltree import Interval, IntervalTree

import cubic_spline_planner
import frenet_optimal_trajectory
import copy




# Make lat lon model as parameter

class DNSMainDecision(object):
    def __init__(self):
        self.dynamic_map = None
        self.vehicle_list = []
        self.pedestrian_list = []

    # Loop entry
    def update_dynamic_map(self, dynamic_map):
        self.dynamic_map = dynamic_map
        self.vehicle_list = dynamic_map.jmap.obstacles
        # TODO(zyxin): Wait for Carla fix this issue
        # self.vehicle_list = [obj for obj in dynamic_map.jmap.obstacles if obj.cls.classid & ObjectClass.LEVEL_MASK_0 == ObjectClass.VEHICLE]
        # self.pedestrian_list = [obj for obj in dynamic_map.jmap.obstacles if obj.cls.classid & ObjectClass.LEVEL_MASK_0 == ObjectClass.HUMAN]
    
    # def check_trajectory(self, decision_trajectory, desired_speed):

    #     #decision_trajectory_array = self.convert_trajectory_to_ndarray(decision_trajectory)
    #     #Frenetrefx,Frenetrefy=self.conver_ndarray_to_XY(decision_trajectory_array)

    #     # rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ",decision_trajectory_array[1,0] )#FIXME(nanshan)
    #     # rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefx=[%f] ",Frenetrefx[1] )#FIXME(nanshan)
    #     #
    #     #
    #     # rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ",decision_trajectory_array[1,1] )#FIXME(nanshan)
    #     # rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefy=[%f] ",Frenetrefy[1] )#FIXME(nanshan)
    #     #
    #     # rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ", decision_trajectory_array[-1,0 ])  # FIXME(nanshan)
    #     # rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefx=[%f] ", Frenetrefx[-1])  # FIXME(nanshan)
    #     #
    #     # rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ", decision_trajectory_array[-1, 1])  # FIXME(nanshan)
    #     # rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefy=[%f] ", Frenetrefy[-1])  # FIXME(nanshan)

    #     return False ,desired_speed
   def check_trajectory(self, decision_trajectory, desired_speed):

        decision_trajectory_array = self.convert_trajectory_to_ndarray(decision_trajectory)
        if len(decision_trajectory_array) == 0:
            return False, desired_speed
        self.collision_points = []
        ego_idx = self.get_ego_idx(decision_trajectory_array)
        nearest_idx = len(decision_trajectory_array)-1
        nearest_obstacle = None
        
        # FIXME(zyxin): should be in prediction part
        if self.dynamic_map.model == MapState.MODEL_JUNCTION_MAP or self.dynamic_map is None: 
            for vehicle in self.vehicle_list:
                if vehicle.lane_index > -1:
                    continue
                pred_trajectory = self.pred_trajectory(vehicle)
                collision_idx, collision_time = self.intersection_between_trajectories(decision_trajectory_array,
                                                                                            pred_trajectory,vehicle)
                #TODO:ignore a near distance
                if collision_time < float("inf"):
                    self.collision_points.append((collision_idx,collision_time))

        # TODO: adjust for pedestrain
        # for pedestrian in self.pedestrian_list:
        #     pred_trajectory = self.pred_trajectory(pedestrian)
        #     collision_idx, collision_time = self.intersection_between_trajectories(decision_trajectory_array,pred_trajectory,vehicle)
        #     if collision_idx > ego_idx and collision_idx < nearest_idx:
        #         nearest_idx = collision_idx
        #         nearest_obstacle = pedestrian

        safeguard_speed = self.get_safeguard_speed(ego_idx,decision_trajectory_array,desired_speed)

        if safeguard_speed < desired_speed:
            return True, safeguard_speed
        else:
            return False, desired_speed

    def pred_trajectory(self, obstacle, pred_t=4, resolution=0.1):
        # Assuming constant speed
        loc = np.array([obstacle.state.pose.pose.position.x, obstacle.state.pose.pose.position.y])
        speed = get_speed(obstacle.state)
        yaw = get_yaw(obstacle.state)
        speed_direction = np.array([np.cos(yaw), np.sin(yaw)])
        t_space = np.linspace(0, pred_t, pred_t/resolution)

        pred_x = loc[0] + t_space*speed*speed_direction[0]
        pred_y = loc[1] + t_space*speed*speed_direction[1]
        pred_trajectory = np.array([pred_x, pred_y]).T

        return pred_trajectory

    def intersection_between_trajectories(self, decision_trajectory, pred_trajectory, vehicle, 
                                 collision_thres=4, stop_thres=3, unavoidable_distance = 4):
        '''
        If two trajectory have interection, return the distance
        TODO: Implement this method into a standalone library
        '''
        
        nearest_idx = len(decision_trajectory)-1
        collision_time = float("inf")
        surrounding_vehicle_speed = get_speed(vehicle.state)

        # the object stops
        # if np.linalg.norm(pred_trajectory[0] - pred_trajectory[-1]) < stop_thres:
        if surrounding_vehicle_speed < stop_thres:
            return nearest_idx, collision_time

        ego_pose = self.dynamic_map.ego_state.pose.pose
        ego_loc = np.array([ego_pose.position.x,ego_pose.position.y])

        for wp in pred_trajectory:
            dist = np.linalg.norm(decision_trajectory - wp,axis=1)
            min_d = np.min(dist)
            distance_to_ego_vehicle = np.linalg.norm(wp - ego_loc)
            if distance_to_ego_vehicle < unavoidable_distance:
                break

            if min_d < collision_thres:
                nearest_idx = np.argmin(dist)
                surrounding_vehicle_distance_before_collision = dist_from_point_to_polyline2d(wp[0],wp[1],pred_trajectory)[2]
                collision_time = surrounding_vehicle_distance_before_collision/surrounding_vehicle_speed
                # rospy.logdebug("considering vehicle:%d, dis_to_collision:%d, col_time:%.2f, speed:%.2f col_index:%d(%d)",vehicle.uid,
                #                                                     surrounding_vehicle_distance_before_collision,
                #                                                     collision_time,surrounding_vehicle_speed,
                #                                                     nearest_idx,len(decision_trajectory))
                break
            
        return nearest_idx,collision_time
        

    def convert_trajectory_to_ndarray(self, trajectory):

        trajectory_array = [(pose.pose.position.x, pose.pose.position.y) for pose in trajectory.poses]
        return np.array(trajectory_array)


    def get_ego_idx(self, decision_trajectory):
        ego_pose = self.dynamic_map.ego_state.pose.pose
        ego_loc = np.array([ego_pose.position.x,ego_pose.position.y])
        dist = np.linalg.norm(decision_trajectory-ego_loc, axis=1)
        ego_idx = np.argmin(dist)
        return ego_idx

    def get_safeguard_speed(self, ego_idx, decision_trajectory, desired_speed, pred_t = 4, safe_dis_range = 7):
        
        danger_speed_range = IntervalTree()
        for collision_idx, collision_time in self.collision_points:
            if collision_idx>len(decision_trajectory)-1:
                continue
            dis_sum = np.cumsum(np.linalg.norm(np.diff(decision_trajectory, axis=0), axis=1))
            dis = dis_sum[collision_idx-1] - dis_sum[ego_idx]

            speed_min = max(0,((dis-safe_dis_range)/collision_time))
            speed_max = min(desired_speed+0.01,(dis+safe_dis_range)/collision_time)
            if speed_min >= speed_max:
                continue
            # rospy.logdebug("col_idx:%d,col_time:%.2f,ocp_min:%.2f,ocp_max:%.2f,desired_speed:%.2f)",
            #                                                                 collision_idx,collision_time,
            #                                                                 speed_min,speed_max,desired_speed)
            danger_speed_range[speed_min:speed_max]=(speed_min,speed_max)


        if len(danger_speed_range[desired_speed]) == 0:
            return desired_speed
        else:
            danger_speed_range.merge_overlaps()
            speed = sorted(danger_speed_range)[-1].begin
            return speed
        

    def convert_trajectory_to_ndarray(self, trajectory):

        trajectory_array = [(pose.pose.position.x, pose.pose.position.y) for pose in trajectory.poses]
        return np.array(trajectory_array)
    
    def conver_ndarray_to_XY(self,path): #FIXME(nanshan 后续和上面的函数合成一个)
        global_x=[]
        global_y=[]
        for wp in path:
            global_x.append(wp[0])
            global_y.append(wp[1])
        return np.array(global_x), np.array(global_y)

    # def generate_trajectory_with_speed(self, dynamic_map):

    #     # update_dynamic_local_map
    #     self.dynamic_map = dynamic_map

    #     changing_lane_index, desired_speed = self.lateral_model_instance.lateral_decision(dynamic_map)
    #     if desired_speed < 0:
    #         desired_speed = 0
    #     rospy.logdebug("target_lane_index = %d, target_speed = %f km/h", changing_lane_index, desired_speed*3.6)

    #     # TODO: Is this reasonable?
    #     # if len(self.dynamic_map.jmap.reference_path.map_lane.central_path_points) == 0:
    #     #     return DecisionTrajectory() # Return null trajectory

    #     # get trajectory by target lane and desired speed
    #     trajectory = self.get_trajectory(changing_lane_index, desired_speed)
    #     ########dns
    #     Frenetrefx,Frenetrefy=self.conver_ndarray_to_XY(trajectory)
        
    #     rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ",trajectory[1,0] )#FIXME(nanshan)
    #     rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefx=[%f] ",Frenetrefx[1] )#FIXME(nanshan)


    #     rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ",trajectory[1,1] )#FIXME(nanshan)
    #     rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefy=[%f] ",Frenetrefy[1] )#FIXME(nanshan)

    #     rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ", trajectory[-1,0 ])  # FIXME(nanshan)
    #     rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefx=[%f] ", Frenetrefx[-1])  # FIXME(nanshan)

    #     rospy.loginfo("DNSSSSSSSS: i am in node trajectory111=[%f] ", trajectory[-1, 1])  # FIXME(nanshan)
    #     rospy.loginfo("DNSSSSSSSS: i am in node FreFrenetrefy=[%f] ", Frenetrefy[-1])  # FIXME(nanshan)

    #     tx, ty, tyaw, tc, csp=frenet_optimal_trajectory.generate_target_course(Frenetrefx,Frenetrefy)

    #     ob = np.array([
    #             [100,-8]
    #             ])
    #     c_speed = desired_speed # current speed [m/s]
    #     c_d = 0  # current lateral position [m]
    #     c_d_d = 0.0  # current lateral speed [m/s]
    #     c_d_dd = 0.0  # current latral acceleration [m/s]
    #     s0 = 0.0  # current course position
    #     #rospy.logdebug("Frenetrefx 1 = %f, Frenetrefy = %f ", Frenetrefx[0], Frenetrefy[0])

    #     bestpath=frenet_optimal_trajectory.frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
    #     rospy.loginfo("DNSSSSSSSS: i am in node besxxxxxxxxx1=[%f] ", bestpath.x[-1])  # FIXME(nanshan)
    #     rospy.loginfo("DNSSSSSSSS: i am in node besyyyyyyyyyy=[%f] ", bestpath.y[-1])  # FIXME(nanshan)
    #     msg = DecisionTrajectory()
    #     msg.trajectory=self.convert_XY_to_pathmsg(bestpath.x,bestpath.y)

    #     ########


    #     #msg.trajectory = self.convert_ndarray_to_pathmsg(trajectory) # TODO: move to library
    #     msg.desired_speed = desired_speed

    #     return msg

    # def get_trajectory(self, target_lane_index, desired_speed, resolution=0.5,
    #                                 time_ahead=5, distance_ahead=10, rectify_thres=2,
    #                                 lc_dt = 1.5, lc_v = 2.67):
    #     # TODO: get smooth spline (write another module to generate spline)
    #     ego_x = self.dynamic_map.ego_state.pose.pose.position.x
    #     ego_y = self.dynamic_map.ego_state.pose.pose.position.y
    #     target_lane = self.get_lane_by_index(target_lane_index)

    #     central_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)

    #     # if ego vehicle is on the target path
    #     # works for lane change, lane follow and reference path follow
    #     dense_centrol_path = dense_polyline2d(central_path, resolution)
    #     #nearest_dis, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_centrol_path)
    #     #nearest_dis = abs(nearest_dis)
    #     nearest_idx = 0
    #     nearest_dis = 0
    #     for i,wp in enumerate(dense_centrol_path):
    #         d = math.sqrt(((wp[0]-ego_x)*(wp[0]-ego_x)+(wp[1]-ego_y)*(wp[1]-ego_y)))
    #         if d < 1:
    #             nearest_idx = i
    #             nearest_dis = d
    #             break
                
    #     rospy.logdebug("nearest_idx = %d, nearest_dis = %f m, total_length = %d", nearest_idx, nearest_dis, len(dense_centrol_path))
    #     if nearest_dis > rectify_thres:
    #         if self.dynamic_map.model == MapState.MODEL_MULTILANE_MAP and target_lane_index != -1:
    #             rectify_dt = abs(self.dynamic_map.mmap.ego_lane_index - target_lane_index)*lc_dt
    #         else:
    #             rectify_dt = nearest_dis/lc_v
    #         return self.generate_smoothen_lane_change_trajectory(target_lane,rectify_dt,desired_speed)

    #     else:
    #         front_path = dense_centrol_path[nearest_idx:]
    #         dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))
    #         trajectory = front_path[:np.searchsorted(dis_to_ego, desired_speed*time_ahead+distance_ahead)-1]
    #         return trajectory

    # def get_lane_by_index(self,lane_index):

    #     if lane_index == -1:
    #         return self.dynamic_map.jmap.reference_path
    #     else:
    #         return self.dynamic_map.mmap.lanes[int(lane_index)]

    #     return None

    # # TODO(zyxin): Add these to zzz_navigation_msgs.utils
    # def convert_path_to_ndarray(self, path):
    #     point_list = [(point.position.x, point.position.y) for point in path]
    #     return np.array(point_list)
    
    # def convert_ndarray_to_pathmsg(self, path, path_id = 'map'):
    #     msg = Path()
    #     for wp in path:
    #         pose = PoseStamped()
    #         pose.pose.position.x = wp[0]
    #         pose.pose.position.y = wp[1]
    #         msg.poses.append(pose)
    #     msg.header.frame_id = path_id # FIXME(NANSHAN):
    #     return msg

    # def convert_XY_to_pathmsg(self,XX,YY,path_id = 'map'):
    #     msg = Path()
    #     for i in range(1,len(XX)):
    #         pose = PoseStamped()
    #         pose.pose.position.x = XX[i]
    #         pose.pose.position.y = YY[i]
    #         msg.poses.append(pose)
    #     msg.header.frame_id = path_id 
    #     return msg

    # def conver_ndarray_to_XY(self,path): #FIXME(nanshan )
    #     global_x=[]
    #     global_y=[]
    #     for wp in path:
    #         global_x.append(wp[0])
    #         global_y.append(wp[1])
    #     return np.array(global_x), np.array(global_y)

    # def generate_smoothen_lane_change_trajectory(self, target_lane, rectify_dt, desired_speed,
    #                                                     lc_dt = 1.5, rectify_min_d = 6, resolution=0.5, time_ahead=5, distance_ahead=10):

    #     target_lane_center_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)


    #     ego_x = self.dynamic_map.ego_state.pose.pose.position.x
    #     ego_y = self.dynamic_map.ego_state.pose.pose.position.y

    #     # Calculate the longitudinal distance for lane Change
    #     # Considering if the ego_vehicle is in a lane Change
    #     lc_dis = max(rectify_dt*desired_speed,rectify_min_d)

    #     dense_target_centrol_path = dense_polyline2d(target_lane_center_path, resolution)
    #     _, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_target_centrol_path)
    #     front_path = dense_target_centrol_path[nearest_idx:]
    #     dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))

    #     start_point = np.array([ego_x,ego_y])
    #     end_point = front_path[np.searchsorted(dis_to_ego, lc_dis)]

    #     # calculate start direction and end direction for control

    #     ego_direction = get_yaw(self.dynamic_map.ego_state)
    #     _, nearest_end_idx, _ = dist_from_point_to_polyline2d(end_point[0], end_point[1], target_lane_center_path)
    #     end_point_direction = target_lane.map_lane.central_path_points[nearest_end_idx].tangent

    #     start_tangent = np.array([np.cos(ego_direction),np.sin(ego_direction)])
    #     end_tangent = np.array([np.cos(end_point_direction),np.sin(end_point_direction)])

    #     lc_path = self.cubic_hermite_spline(p0 = start_point, p1 = end_point,
    #                                         m0 = start_tangent, m1 = end_tangent)

    #     # get ahead Path
    #     ahead_dis = desired_speed*time_ahead+distance_ahead
    #     path_after_lc = front_path[np.searchsorted(dis_to_ego, lc_dis):np.searchsorted(dis_to_ego, ahead_dis)-1]

    #     # replace lane change path into ahead path
    #     smoothen_lc_path = np.concatenate((lc_path,path_after_lc),axis = 0)

    #     return smoothen_lc_path

    # def cubic_hermite_spline(self, p0, p1, m0, m1, resolution = 20):
    #     """
    #     Generate cubic hermit spline
    #     p0: start point np.array(2)
    #     p1: end point np.array(2)
    #     m0: start tangent np.array(2)
    #     m1: end tangent np.array(2)
    #     return path from start point to end point
    #     """

    #     t = np.linspace(0,1,num = resolution)
    #     h00 = (2*t*t*t-3*t*t+1).reshape(len(t),1) #p0
    #     h10 = (t*t*t-2*t*t+t).reshape(len(t),1) #m0
    #     h01 = (-2*t*t*t+3*t*t).reshape(len(t),1) #p1
    #     h11 = (t*t*t-t*t).reshape(len(t),1) #m1

    #     p0 = p0.reshape(1,2)
    #     p1 = p1.reshape(1,2)
    #     m0 = m0.reshape(1,2)
    #     m1 = m1.reshape(1,2)

    #     return np.matmul(h00,p0) + np.matmul(h10,m0) + np.matmul(h01,p1) + np.matmul(h11,m1)


    # def get_reference_buffer_msg(self):

    #     ego_x = self.dynamic_map.ego_state.pose.pose.position.x
    #     ego_y = self.dynamic_map.ego_state.pose.pose.position.y
    #     target_lane = -1

    #     central_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)

    #     msg = self.convert_ndarray_to_pathmsg(central_path)


