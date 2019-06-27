#!/usr/bin/env python

import numpy as np

class MainDecision(object):
    def __init__(self):

        self.reference_path = deque(maxlen=20000)
        self.reference_path_buffer = deque(maxlen=200)
        self.trajectory = deque()
        self.ego_vehicle_location = np.array([0,0])
        self.judge_LC_thr = 3

    def setup(self):

        pass

    def setup_reference_path(self,reference_path):
        self.reference_path.clear()
        for i, wp in enumerate(reference_path.poses):
            self.reference_path.append(np.array([wp.pose.position.x,-wp.pose.position.y]))

    """
    Main
    """

    def update_trajectory(self,ego_x,ego_y):
        """
        The trajectory is used for control (main)
        """
        self.ego_vehicle_location = np.array([ego_x,ego_y])

        # update reference path
        self.update_reference_path_buffer()

        # in junction (or near) -> follow the reference path
        if self.should_follow_the_reference_path():
            trajectory = self.get_trajectory_from_reference_path()
        else:
            # in multi-lane scenarios -> follow the central path / lane change
            trajectory = self.get_trajectory_from_decision_module()

        # Publish the generate trajectory
        return trajectory

    """
    Reference Path
    """

    def update_reference_path_buffer(self):

        # find the nearest point
        min_distance = 50
        min_index = 0
        index = 0

        while index < 40 and index < len(self.reference_path_buffer) and self.reference_path_buffer[index]:
            d = np.linalg.norm(self.reference_path_buffer[index]-self.ego_vehicle_location)
            if d < min_distance:
                min_distance = d
                min_index = index
            index += 1

        for index in range(0, min_index):
            self.reference_path_buffer.popleft()

        while self.reference_path and len(self.reference_path_buffer) < self._buffer_size:
            wp = self.reference_path.popleft()
            self.lane_change_smoothen(wp)
            self.reference_path_buffer.append(wp)


    def lane_change_smoothen(self,wp):

        if self.reference_path_buffer:
            last_wp = self.reference_path_buffer[-1]
        else:
            return
        if np.linalg.norm(last_wp-wp) < self.judge_lc_thr:
            return

        ## Start smoothen the reference path
        rospy.logdebug("Reference Path Smoothing")
        lane_change_distance = min(20,len(self.reference_path_buffer)) # TODO
        last_wp = wp
        first_wp = self.reference_path_buffer[-lane_change_distance]
        loc_xs = np.linspace(last_wp[0], first_wp[0],num = lane_change_distance+1)
        loc_ys = np.linspace(last_wp[1], first_wp[1],num = lane_change_distance+1)
        for i in range(1,lane_change_distance):
            self.reference_path_buffer[-1] = np.array([loc_xs[i],loc_ys[i]])  

    def get_trajectory_from_reference_path(self):
        
        self.trajectory.clear
        for i in range(0,min(50,len(self.reference_path_buffer))):
            self.trajectory.append(self.reference_path_buffer[i])

    """
    Decision Module
    """
    
    


    
