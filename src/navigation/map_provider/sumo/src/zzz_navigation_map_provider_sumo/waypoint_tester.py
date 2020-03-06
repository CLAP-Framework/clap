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
import math
import time
import threading 

from threading import Thread

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from zzz_driver_msgs.msg import RigidBodyStateStamped


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

        self.current_route = np.loadtxt('/home/yli/inner_piece.dat')
        self.current_outer = np.loadtxt('/home/yli/outer_piece.dat')

        self.waypoint_publisher = rospy.Publisher(
            '/carla/{}/waypoints'.format(self.role_name), Path, queue_size=1, latch=True)

        # self._pose_publisher = rospy.Publisher(params.pose_output_topic, RigidBodyStateStamped, queue_size=1)
        self._pose_publisher = rospy.Publisher('/zzz/navigation/ego_pose', RigidBodyStateStamped, queue_size=1)
        self._thread = threading.Thread(target=self.run)
        self._thread.start()

    def run(self):
        flag = False;
        i = 0
        while True:
            time.sleep(1)
            if flag is True:
                wp = self.current_route[i]
            else:
                wp = self.current_outer[i]

            self.publish_ego_pose(wp[0], wp[1])
            if flag is False:
                flag = True
            else:
                flag = False
            i = i+1


    def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for wp in self.current_route:
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

    # wait for ros-bridge to set up CARLA world


    try:

        rospy.loginfo("Connected to Carla.")

        waypointConverter = CarlaToRosWaypointConverter()
        waypointConverter.publish_waypoints()

        rospy.spin()

    finally:
        del waypointConverter
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
