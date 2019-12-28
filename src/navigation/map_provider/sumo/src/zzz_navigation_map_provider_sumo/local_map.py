

import os, sys
import io
import subprocess
from collections import deque
import tempfile
import math, time, random

import rospy
import numpy as np
from zzz_navigation_msgs.msg import Lane, LanePoint, Map
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    rospy.logerr("Please declare environment variable 'SUMO_HOME' (e.g. /usr/share/sumo)")
    sys.exit(-1)
import sumolib

class LocalMap(object):
    def __init__(self, offset_x=0, offset_y=0):
        self._ego_vehicle_x = None # location in meters
        self._ego_vehicle_y = None

        self._hdmap = None # HDMap instance loaded from SUMO

        self._offset_x = offset_x # map center offset in meters
        self._offset_y = offset_y

        self._current_edge_id = None # road id string
        self._reference_lane_list = deque(maxlen=20000)
        self._lane_search_radius = 4

    def setup_hdmap(self, file=None, content=None, mtype=None):
        if not (file or content):
            rospy.logerr("Neither map file nor map content are specified! Map will not be loaded!")
            return False
        rospy.logdebug("Map Type: %s", mtype)
        if mtype == 'sumo':
            if content:
                self._hdmap = sumolib.net.readNet(io.StringIO(content))
            else: # use file
                self._hdmap = sumolib.net.readNet(file)
            self._offset_x, self._offset_y = self._hdmap.getLocationOffset()
            return True

        elif content:
            file_handle = tempfile.NamedTemporaryFile(delete=False)
            file = file_handle.name
            file_handle.write(content)
            file_handle.close()
            # use random number here to prevent multiple instances conflict
            converted_file = os.path.join(tempfile.gettempdir(), "sumo_map_%d.temp" % (random.randint(0, 999)))
            file_created = True
        assert os.path.exists(file)
        if mtype == 'unknown':
            rospy.logerr("Cannot load map with unknown type")
        elif mtype == 'opendrive':
            command = ['netconvert', "--opendrive-files", file,
                "--opendrive.import-all-lanes", "true",
                "--offset.disable-normalization", "true",
                "--opendrive.curve-resolution", "0.5",
                "-o", converted_file]
            command = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
            exitcode = command.wait()
            if exitcode != 0:
                rospy.logerr("SUMO netconvert failed, (exit code: %d). You can try to upgrade SUMO to latest version!" % exitcode)
            else:
                rospy.logdebug("SUMO netconvert succeed, stderr:\n %s", command.stderr.read())

        self._hdmap = sumolib.net.readNet(converted_file)
        self._offset_x, self._offset_y = self._hdmap.getLocationOffset()

        # Clean temporary files
        if file_created:
            os.remove(file)
        os.remove(converted_file)
        return True

    def setup_reference_lane_list(self, reference_path):
        '''
        Generate a list of ids of all lane of the reference path
        '''
        if self._hdmap is None:
            return False

        self._reference_lane_list.clear()
        # TODO: this loop needs to be optimised later

        for wp in reference_path.poses:
            # TODO: takes too much time for processing
            wp_map_x, wp_map_y = self.convert_to_map_XY(wp.pose.position.x, wp.pose.position.y)

            # Find the closest lane of the reference path points
            lanes = self._hdmap.getNeighboringLanes(wp_map_x, wp_map_y, self._lane_search_radius, includeJunctions=False)
            if len(lanes) > 0:
                _, closestLane = min((dist, lane) for lane, dist in lanes)
                # Discard duplicate lane ids
                if len(self._reference_lane_list) != 0 and closestLane.getID() == self._reference_lane_list[-1].getID():
                    continue
                self._reference_lane_list.append(closestLane)

        return True

    def convert_to_map_XY(self, x, y):
        map_x = x + self._offset_x
        map_y = y + self._offset_y
        return map_x, map_y

    def convert_to_origin_XY(self, map_x, map_y):
        x = map_x - self._offset_x
        y = map_y - self._offset_y
        return x,y

    def receive_new_pose(self, x, y):
        self._ego_vehicle_x = x
        self._ego_vehicle_y = y
        if self.should_update_static_map():
            self.update_static_map()
            return self.static_local_map
        return None

    def should_update_static_map(self):
        '''
        Determine whether map updating is needed.
        '''
        map_x, map_y = self.convert_to_map_XY(self._ego_vehicle_x, self._ego_vehicle_y)
        rospy.logdebug("Check update: ego_map_location = (%f, %f), ego_location = (%f, %f)",
            map_x, map_y, self._ego_vehicle_x, self._ego_vehicle_y)

        lanes = self._hdmap.getNeighboringLanes(map_x, map_y, self._lane_search_radius, includeJunctions=False)
        if len(lanes) > 0:
            _, closestLane = min((dist, lane) for lane, dist in lanes)
            new_edge = closestLane.getEdge()
            new_edge.id = new_edge.getID()
            rospy.logdebug("Found ego vehicle neighbor edge id = %s",new_edge.id)
            if self._current_edge_id is None or new_edge.id != self._current_edge_id:
                rospy.loginfo("Should update static map, edge id %s -> %s", self._current_edge_id, new_edge.id)
                return True
        return False

    def init_static_map(self):
        '''
        Generate null static map
        '''
        init_static_map = Map()
        init_static_map.in_junction = True
        init_static_map.target_lane_index = -1
        return init_static_map

    def update_static_map(self):
        ''' 
        Update information in the static map if current location changed dramatically
        '''
        rospy.logdebug("Updating static map")
        self.static_local_map = self.init_static_map() ## Return this one
        self.update_lane_list()
        self.update_target_lane()

        if not self.static_local_map.in_junction:
            self.calibrate_lane_index() # make the righest lane index 0

        rospy.loginfo("Updated static map info: lane_number = %d, in_junction = %d, current_edge_id = %s, target_lane_index = %s",
            len(self.static_local_map.lanes), int(self.static_local_map.in_junction),
            self._current_edge_id, self.static_local_map.target_lane_index)

    def update_lane_list(self):
        '''
        Update lanes when a new road is encountered
        '''
        map_x, map_y = self.convert_to_map_XY(self._ego_vehicle_x, self._ego_vehicle_y)
        lanes = self._hdmap.getNeighboringLanes(map_x, map_y, self._lane_search_radius, includeJunctions=False)
        if len(lanes) > 0:
            self.static_local_map.in_junction = False

            _, closestLane = min((dist, lane) for lane, dist in lanes)
            self.new_lane = closestLane
            self.new_edge = closestLane.getEdge()

            self._current_edge_id = self.new_edge.getID()
            lanes_in_edge = self.new_edge.getLanes()
            for lane in lanes_in_edge:
                connections_outgoing = lane.getOutgoing()
                # Remove fake lane, TODO(zyxin): Remove using SUMO properties (green verge lanes, http://sumo.sourceforge.net/pydoc/sumolib.net.lane.html)
                if len(connections_outgoing) < 1:
                    continue
                lane_wrapped = self.wrap_lane(lane)
                self.static_local_map.lanes.append(lane_wrapped)

    def wrap_lane(self, lane):
        '''
        Wrap lane information into ROS message
        '''
        lane_wrapped = Lane()
        lane_wrapped.index = lane.getIndex()
        last_x = last_y = last_s = None
        for wp in lane.getShape():
            point = LanePoint()
            x, y = self.convert_to_origin_XY(wp[0], wp[1])
            # Calculate mileage
            if last_s is None:
                point.s = 0
            else:
                point.s = last_s + math.sqrt((x-last_x)*(x-last_x) + (y-last_y)*(y-last_y))
            point.position.x = x
            point.position.y = y
            point.width = lane.getWidth()
            # TODO: add more lane point info

            # Update
            last_s = point.s
            last_x = point.position.x
            last_y = point.position.y
            lane_wrapped.central_path_points.append(point)

        return lane_wrapped

    def update_target_lane(self):
        # Skip passed lanes
        while self._reference_lane_list and self._reference_lane_list[0].getEdge().getID() != self._current_edge_id:
            rospy.logdebug("Delete a passed edge: %s, current edge = %s",
                self._reference_lane_list[0].getEdge().getID(), self._current_edge_id)
            self._reference_lane_list.popleft()

        # Find the lane in the next edge
        while self._reference_lane_list and self._reference_lane_list[0].getEdge().getID() == self._current_edge_id:
            rospy.logdebug("Delete lane with current edge id: %s, current edge = %s",
                self._reference_lane_list[0].getEdge().getID(), self._current_edge_id)
            self._reference_lane_list.popleft() # TODO(zyxin): Check if the logic is correct here

        if self._reference_lane_list:
            target_lane_id = self._reference_lane_list[0].getID()
            rospy.logdebug("Detected next lane id: %s",target_lane_id)

            lanes_in_edge = self.new_edge.getLanes()
            for lane in lanes_in_edge:
                connections_outgoing = lane.getOutgoing()
                for connection in connections_outgoing:
                    if connection.getToLane().getID() == target_lane_id:
                        self.static_local_map.target_lane_index = lane.getIndex()
                        rospy.logdebug("Finded next target lane id = %s", self.static_local_map.target_lane_index)
                        return

        # FIXME(zhcao): reference path match target lane
        rospy.logerr("Cannot find next target lane")

    def calibrate_lane_index(self):
        '''
        Makes the drivable lanes starts from 0
        '''
        # TODO(zhcao):should consider ego vehicle lane
        first_index = self.static_local_map.lanes[0].index
        for lane in self.static_local_map.lanes:
            lane.index = lane.index - first_index

        if self.static_local_map.target_lane_index >= 0:
            self.static_local_map.target_lane_index = self.static_local_map.target_lane_index - first_index
