#!/usr/bin/env python

import rospy
import numpy as np
from zzz_common.geometry import *
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from reachable_set import ReachableSet

class Safeguard(object):
    def __init__(self):

        self._safeguard_instance = ReachableSet()

    def update_dynamic_map(self,dynamic_map):
        self._safeguard_instance.update_dynamic_map(dynamic_map)

    def update_vehicle_list(self,vehicle_list):
        self._safeguard_instance.update_vehicle_list(vehicle_list)
        
    def update_pedestrian_list(self,pedestrian_list):
        self._safeguard_instance.update_pedestrian_list(pedestrian_list)
        
    def check_trajectory(self,decision):
        
        safeguard_triggered, safespeed = self._safeguard_instance.check_trajectory(decision.trajectory,decision.desired_speed)

        if safeguard_triggered:
            rospy.loginfo("safeguarded triggered")

        msg = decision
        msg.trajectory = decision.trajectory
        msg.desired_speed = safespeed

        return msg

    