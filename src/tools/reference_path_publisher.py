#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Generates a plan of waypoints to follow

It uses the current pose of the ego vehicle as starting point. If the
vehicle is respawned, the route is newly calculated.

The goal is either read from the ROS topic `/carla/<ROLE NAME>/move_base_simple/goal`, if available
(e.g. published by RVIZ via '2D Nav Goal') or a fixed spawnpoint is used.

The calculated route is published on '/carla/<ROLE NAME>/waypoints'
"""
import os
import sys
import math
import time
import threading 
import signal
import copy

import argparse



from threading import Thread

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import Lane
from autoware_msgs.msg import Waypoint

# from zzz_driver_msgs.msg import RigidBodyStateStamped
#pub = rospy.Publisher('/lane_waypoints_array', Lane, queue_size=1, latch=True)


# cmd line argument parser, help
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('--map', required=True, help=' ${ your_map_dat_file }')

args = arg_parser.parse_args()

def path_callback(data):
    """
    callback for path. Convert it to Autoware LaneArray and publish it
    """
    msg = LaneArray()
    lane = Lane()
    lane.header = data.header
    msg.lanes.append(lane)
    for pose in data.poses:
        waypoint = Waypoint()
        waypoint.pose = pose
        waypoint.twist.twist.linear.x = 4.0 # predefine speed
        msg.lanes[0].waypoints.append(waypoint)

    pub.publish(msg)

def path_callback2(data):
    """
    callback for path. Convert it to Autoware LaneArray and publish it
    """
    lane = Lane()
    lane.header = data.header
    for pose in data.poses:
        waypoint = Waypoint()
        waypoint.pose = pose
        waypoint.twist.twist.linear.x = 4.0 # predefine speed
        lane.waypoints.append(waypoint)
    rospy.loginfo("++++ convert_waypoints_carla_to_autoware waypoints {} ++++".format(len(lane.waypoints)))
    pub.publish(lane)


interrupt = False

def quit(signum, frame):
    interrupt = True
    print 'You choose to stop me.'
    sys.exit()

class CarlaToRosWaypointConverter(object):
    """
    read map then load it into routes, 
    published to real car's mini-auto planning module
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self):

        self.ego_vehicle = None
        self.role_name = 'ego_vehicle' 

        self._ego_pose_x = 0.0
        self._ego_pose_y = 0.0

        # self.current_route = (np.loadtxt(
        #     os.environ.get('MINI_AUTO_ROOT') + '/src/map/data/shougang/map_shougang_square.dat', delimiter=',')).tolist()
        self.current_route = (np.loadtxt(args.map, delimiter=',')).tolist()

        self.waypoint_publisher = rospy.Publisher('/lane_waypoints_array', LaneArray, queue_size=1, latch=True)
        # rospy.Publisher(
        #     '/carla/{}/waypoints'.format(self.role_name), Path, queue_size=1, latch=True)

        # self._pose_subscriber = rospy.Subscriber('/zzz/navigation/ego_pose', RigidBodyStateStamped, self.pose_callback)
        # self._thread = threading.Thread(target=self.run)
        # self._thread.start()
        
    def pose_callback(self, msg):
        # Note: Here we actually assume that pose is updating at highest frequency
        self._ego_pose_x, self._ego_pose_y = msg.state.pose.pose.position.x, msg.state.pose.pose.position.y
        # rospy.loginfo('+++ {},{} +++'.format(self._ego_pose_x, self._ego_pose_y))


    def rebuild_routes(self):

        dist_list = np.linalg.norm(
            np.array(self.current_route) - [self._ego_pose_x, self._ego_pose_y],
            axis = 1)

        max = np.argmax(dist_list)

        new_route = []
        for i in self.current_route[max : -1]:
            new_route.append(i)
             
        for i in self.current_route[0: max-1]:
            new_route.append(i)

        rospy.loginfo('### {} ###'.format(new_route[0]))
        return new_route


    def run(self):
        while True:
            new_route = self.rebuild_routes()
            self.publish_waypoints(new_route)
            time.sleep(3)

    def publish_path(self):
        self.publish_waypoints(self.current_route)

    def publish_waypoints(self, new_route):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for wp in new_route:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            # orientation information from original values from oxford gps module.!!!!
            pose.pose.orientation.x = wp[2]
            pose.pose.orientation.y = wp[3]
            pose.pose.orientation.z = wp[4]
            pose.pose.orientation.w = wp[5]
            msg.poses.append(pose)

        self.republish_lane_array(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))

    def republish_lane_array(self, data):
        msg = LaneArray()
        lane = Lane()
        lane.header = data.header
        msg.lanes.append(lane)
        for pose in data.poses:
            waypoint = Waypoint()
            waypoint.pose = pose
            waypoint.twist.twist.linear.x = 4.0 # predefine speed
            msg.lanes[0].waypoints.append(waypoint)
        self.waypoint_publisher.publish(msg)


def main():
    """
    main function
    """
    rospy.init_node("reference_path_publisher", anonymous=True)

    signal.signal(signal.SIGINT, quit)

    # wait for ros-bridge to set up CARLA world
    try:
        rospy.loginfo("!!! Reference Path Publisher Initialized !!!")
        waypointConverter = CarlaToRosWaypointConverter()
        waypointConverter.publish_path()
        rospy.spin()

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
