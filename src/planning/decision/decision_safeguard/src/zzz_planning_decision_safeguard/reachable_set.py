
import rospy
import numpy as np


class ReachableSet(object):

    def __init__(self,longitudinal_model):
        self.dynamic_map = None
        self.vehicle_list = []
        self.pedestrian_list = []

    def update_dynamic_map(dynamic_map):
        self.dynamic_map = dynamic_map

    def update_vehicle_list(vehicle_list):
        self.vehicle_list = vehicle_list
    
    def update_pedestrian_list(pedestrian_list):
        self.pedestrian_list = pedestrian_list
    
    def check_trajectory(self,decision_trajectory,desired_speed):

        decision_trajectory_array = self.convert_trajectory_to_ndarray(decision_trajectory)
        ego_idx = self.get_ego_idx(decision_trajectory_array)
        nearest_idx = len(decision_trajectory_array)
        nearest_obstacle = None

        for vehicle in self.vehicle_list:
            pred_trajectory = self.pred_trajectory(vehicle)
            collision_idx = self.intersection_between_trajectories(decision_trajectory_array,pred_trajectory)
            #TODO:ignore a near distance
            if collision_idx > ego_idx and collision_idx < nearest_idx:
                nearest_idx = collision_idx
                nearest_obstacle = vehicle

        for pedestrian in self.pedestrian_list:
            pred_trajectory = self.pred_trajectory(pedestrian)
            collision_idx = self.intersection_between_trajectories(decision_trajectory_array,pred_trajectory)
            if collision_idx > ego_idx and collision_idx < nearest_idx:
                nearest_idx = collision_idx
                nearest_obstacle = pedestrian
        
        safeguard_speed = self.get_safeguard_speed(ego_idx,nearest_idx,decision_trajectory_array)

        if nearest_obstacle is None or safeguard_speed > desired_speed:
            return False, desired_speed
        else:
            return True, safeguard_speed

    def pred_trajectory(self,obstacle,pred_t = 4,resolution = 0.5):
        # obstacle could be pedestrian or vehicle

        loc = np.array([obstacle.obstacle_pos_x,obstacle.obstacle_pos_y])
        speed = obstacle.obstacle_speed
        yaw = obstacle.obstacle_yaw
        speed_direction = np.array([math.cos(math.radians(yaw)),math.sin(math.radians(yaw))])
        t_space = np.linspace(0,pred_t,pred_t/resolution)

        pred_x = loc[0]+t_space*speed*speed_direction[0]
        pred_y = loc[1]+t_space*speed*speed_direction[1]

        pred_trajectory = np.array([pred_x,pred_y]).T

        return pred_trajectory

    def intersection_between_trajectories(self,decision_trajectory,pred_trajectory,collision_thres=4,stop_thres = 5):
        # if two trajectory have interection:
        # return the distance 
        
        nearest_idx = len(decision_trajectory)

        # the object stops
        if np.linalg.norm(pred_trajectory[0]-pred_trajectory[-1]) < stop_thres:
            return nearest_idx

        for i, wp in enumerate(pred_trajectory):
            dist = np.linspace.norm(decision_trajectory-wp,axis=1)
            min_d = np.min(dist)
            if min_d  < collision_thres:
                nearest_idx = np.argmin(dist)
                break
            
        return nearest_idx
        

    def convert_trajectory_to_ndarray(self,trajectory):

        trajectory_array = [(pose.pose.position.x, pose.pose.position.y) for pose in trajectory.poses]
        return np.array(trajectory)


    def get_ego_idx(self,decision_trajectory):
        
        ego_pose = self.dynamic_map.ego_pose
        ego_loc = np.array([ego_pose.position.x,ego_pose.position.y])
        dist = np.linspace.norm(decision_trajectory-ego_loc,axis=1)
        ego_idx = np.argmin(dist)
        return ego_idx

    def get_safeguard_speed(self,ego_idx,collision_idx,decision_trajectory,pred_t = 4):
        dis_sum = np.cumsum(decision_trajectory)
        dis = dis_sum[collision_idx] - dis_sum[ego_idx]
        speed = dis/pred_t
        return speed

        