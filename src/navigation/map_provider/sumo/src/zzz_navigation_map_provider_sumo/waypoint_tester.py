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

from threading import Thread

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from zzz_driver_msgs.msg import RigidBodyStateStamped


def quit(signum, frame):
    print 'You choose to stop me.'
    sys.exit()

class CarlaToRosWaypointConverter(object):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self):

        self.ego_vehicle = None
        self.role_name = 'ego_vehicle' 

        self._ego_vehicle_x = 0.0
        self._ego_vehicle_y = 0.0

        self.current_route = np.loadtxt(
            os.environ.get('ZZZ_ROOT') + '/zzz/src/navigation/data/outer_loop.dat', 
            delimiter=',')

        self.waypoint_publisher = rospy.Publisher(
            '/carla/{}/waypoints'.format(self.role_name), Path, queue_size=1, latch=True)

        self._pose_subscriber = rospy.Subscriber('/zzz/navigation/ego_pose', RigidBodyStateStamped, self.pose_callback)
        self._thread = threading.Thread(target=self.run)
        self._thread.start()

    def pose_callback(self, msg):
        # Note: Here we actually assume that pose is updating at highest frequency
        self._ego_pose_x, self._ego_pose_y = msg.state.pose.pose.position.x, msg.state.pose.pose.position.y


    def rebuild_routes(self):

        central_path = self.current_route

        dist_list = np.linalg.norm(
            central_path - [self._ego_vehicle_x, self._ego_vehicle_y],
            axis = 1)
        max = np.argmax(dist_list)
        
        new_route = []
        for i in central_path[max : -1]:
            new_route.append(i)
             
        for i in central_path[0: max-1]:
            new_route.append(i)

        rospy.loginfo('### {} ###'.format(len(new_route)))
        return new_route


    def run(self):
        while True:
            new_route = self.rebuild_routes()
            self.publish_waypoints(new_route)
            time.sleep(10)


    def publish_waypoints(self, current_route):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for wp in current_route:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))


    def publish_ego_pose(self, x, y):

        # Transformation from odom frame to map is static
        state = RigidBodyStateStamped()
        state.header.frame_id = "map"
        state.state.child_frame_id = "odom"

        state.state.pose.pose.position.x = x
        state.state.pose.pose.position.y = y
        state.state.pose.pose.position.z = 0.0
        state.state.pose.pose.orientation.x = 0.0
        state.state.pose.pose.orientation.y = 0.0
        state.state.pose.pose.orientation.z = 0.0
        state.state.pose.pose.orientation.w = 0.0

        rospy.logdebug("Position: %.3f, %.3f", state.state.pose.pose.position.x, state.state.pose.pose.position.y)
        
        self._pose_publisher.publish(state)


def main():
    """
    main function
    """
    rospy.init_node("waypoint_tester", anonymous=True)

    signal.signal(signal.SIGINT, quit)

    # wait for ros-bridge to set up CARLA world


    try:

        rospy.loginfo("Connected to Carla.")

        waypointConverter = CarlaToRosWaypointConverter()

        rospy.spin()

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
