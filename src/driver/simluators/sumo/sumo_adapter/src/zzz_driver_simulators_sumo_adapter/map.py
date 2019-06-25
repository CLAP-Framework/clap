

import os, sys
from collections import deque

from zzz_driver_msgs.msg import LaneState, MapState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib

class StaticLocalMap(object):
    def __init__(self):
        self.ego_vehicle_location_x = None # location in meters
        self.ego_vehicle_location_y = None
        self.hdmap = None # HDMap instance loaded from SUMO
        self.offset_x = 0.0 # map center offset in meters
        self.offset_y = 0.0
        self.current_edge_id = None # road id string
        self.reference_lane_list = deque(maxlen=20000)

    def setup_hdmap(self, hdmap_filepath):

        self.hdmap = sumolib.net.readNet(hdmap_filepath)
        self.offset_x, self.offset_y = self.hdmap.getLocationOffset()

    def setup_reference_lane_list(self, reference_path):
        '''
        reference_path: Shoud be of type `geometry_msgs.msg.Pose`
        '''

        assert self.hdmap is not None
        self.reference_lane_list.clear()
        for wp in reference_path.poses:
            wp_map_x, wp_map_y = self.convert_to_map_XY(wp.pose.position.x,wp.pose.position.y)
            radius_lane = 1
            lanes = self.hdmap.getNeighboringLanes(wp_map_x, wp_map_y, radius_lane)
            if len(lanes)>0:
                distancesAndLanes = sorted([(dist, lane) for lane, dist in lanes])
                dist, closestLane = distancesAndLanes[0]
                if len(self.reference_lane_list) != 0 and closestLane.getID() == self.reference_lane_list[-1].getID():
                    continue
                self.reference_lane_list.append(closestLane)

    def convert_to_map_XY(self,x,y):

        map_x = x + self.offset_x
        map_y = -y + self.offset_y

        return map_x, map_y

    def convert_to_origin_XY(self,map_x,map_y):

        x = map_x - self.offset_x
        y = -(map_y - self.offset_y)
        return x,y

    def receive_new_pose(self,x,y):

        self.ego_vehicle_location_x = x
        self.ego_vehicle_location_y = y
        if self.should_update_static_map():
            self.update_static_map()
            return self.static_local_map

        return None

    def should_update_static_map(self):

        map_x, map_y = self.convert_to_map_XY(self.ego_vehicle_location_x,self.ego_vehicle_location_y)
        radius_lane = 1
        lanes = self.hdmap.getNeighboringLanes(map_x, map_y, radius_lane)
        if len(lanes) > 0:
            distancesAndLanes = sorted([(dist, lane) for lane, dist in lanes])
            dist, closestLane = distancesAndLanes[0]
            new_edge = closestLane.getEdge()
            new_edge.id = new_edge.getID()
            if self.current_edge_id is None or new_edge.id != self.current_edge_id:
                return True

        return False

    def update_static_map(self):

        self.static_local_map = MapState() ## Return this one
        self.update_lane_list()
        self.update_target_lane()

    def update_lane_list(self):
        
        map_x, map_y = self.convert_to_map_XY(self.ego_vehicle_location_x, self.ego_vehicle_location_y)

        radius_lane = 1
        lanes = self.hdmap.getNeighboringLanes(map_x, map_y, radius_lane)
        if len(lanes) > 0:
            distancesAndLanes = sorted([(dist, lane) for lane, dist in lanes])
            dist, closestLane = distancesAndLanes[0]
            self.new_lane = closestLane
            self.new_edge = closestLane.getEdge()

            self.current_edge_id = self.new_edge.getID()
            lanes_in_edge = self.new_edge.getLanes()
            for lane in lanes_in_edge:
                add_lane = self.wrap_lane(lane)
                self.static_local_map.lanes.append(add_lane)

    def wrap_lane(self,lane):
        
        add_lane = LaneState()
        add_lane.index = lane.getIndex()
        shape = lane.getShape()
        central_path = Path()
        for wp in shape:
            pose = PoseStamped()
            x,y = self.convert_to_origin_XY(wp[0],wp[1])
            pose.pose.position.x = x
            pose.pose.position.y = y
            central_path.poses.append(pose)
        add_lane.central_path = central_path

        return add_lane

    def update_target_lane(self):

        while self.reference_lane_list and self.reference_lane_list[0].getEdge().getID() != self.current_edge_id:
            self.reference_lane_list.popleft()

        while self.reference_lane_list and self.reference_lane_list[0].getEdge().getID() == self.current_edge_id:
            self.reference_lane_list.popleft()

        if self.reference_lane_list:
            target_lane_id = self.reference_lane_list[0].getID()
            lanes_in_edge = self.new_edge.getLanes()
            for lane in lanes_in_edge:
                connections_outgoing = lane.getOutgoing()
                for connection in connections_outgoing:
                    if connection.getToLane().getID() == target_lane_id:
                        self.static_local_map.target_lane_index = lane.getIndex()
                        return
