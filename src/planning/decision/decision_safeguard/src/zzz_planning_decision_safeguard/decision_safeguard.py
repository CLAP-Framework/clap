#!/usr/bin/env python

import rospy
import numpy as np
from zzz_common.geometry import *
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Safeguard(object):
    def __init__(self):

        self.dynamic_map = None
        self.longitudinal_model_instance = LonIDM()
        self.lateral_model_instance = LatLaneUtility(self.longitudinal_model_instance)


    def update_dynamic_map(self):
        pass

    def update_surrounding_vehicle_list(self):
        pass

    def update_surrounding_pedestrian_list(self):
        pass

    def check_trajectory(self):
        
        


        if safeguard_triggered:
            rospy.loginfo("safeguarded triggered")

        return safeguard_triggered, trajectory_msg

    def pred_trajectory(self,obstacle,pred_t = 5,resolution = 0.5):
        # obstacle could be pedestrian or vehicle

        loc = np.array([obstacle.obstacle_pos_x,obstacle.obstacle_pos_y])
        speed = obstacle.obstacle_speed
        t_space = np.linspace(0,pred_t,pred_t/resolution)
        pred_tractory = t_space*speed + loc

        return np.array(pred_tractory)

    def intersection_between_trajectories(self,decision_trajectory,pred_trajectory,collision_thres):
        # if two trajectory have interection:
        # return the distance 
