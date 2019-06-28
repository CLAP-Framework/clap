

import os, sys
from collections import deque

from zzz_driver_msgs.msg import LaneState, MapState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy

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
        self.radius_lane = 4

    def setup_hdmap(self, hdmap_filepath):

        self.hdmap = sumolib.net.readNet(hdmap_filepath)
        self.offset_x, self.offset_y = self.hdmap.getLocationOffset()

    def setup_reference_lane_list(self, reference_path):
        '''
        reference_path: Shoud be of type `geometry_msgs.msg.Pose`
        '''

        assert self.hdmap is not None
        self.reference_lane_list.clear()
        for i, wp in enumerate(reference_path.poses):
            wp_map_x, wp_map_y = self.convert_to_map_XY(wp.pose.position.x,-wp.pose.position.y)

            lanes = self.hdmap.getNeighboringLanes(wp_map_x, wp_map_y, self.radius_lane, includeJunctions=False)
            if len(lanes)>0:
                distancesAndLanes = sorted([(dist, lane) for lane, dist in lanes])
                dist, closestLane = distancesAndLanes[0]
                if len(self.reference_lane_list) != 0 and closestLane.getID() == self.reference_lane_list[-1].getID():
                    continue
                self.reference_lane_list.append(closestLane)
                # print(closestLane.getID())

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

        lanes = self.hdmap.getNeighboringLanes(map_x, map_y, self.radius_lane,includeJunctions=False)
        rospy.logdebug("ego_map_location = %f , %f, ego_location = %f, %f",map_x,map_y,self.ego_vehicle_location_x,self.ego_vehicle_location_y)
        if len(lanes) > 0:
            distancesAndLanes = sorted([(dist, lane) for lane, dist in lanes])
            dist, closestLane = distancesAndLanes[0]
            new_edge = closestLane.getEdge()
            new_edge.id = new_edge.getID()
            rospy.logdebug("find a neighbor edge id = %s",new_edge.id)
            if self.current_edge_id is None or new_edge.id != self.current_edge_id:
                rospy.loginfo("Should update static map, edge id %s -> %s", self.current_edge_id, new_edge.id)
                return True

        rospy.logdebug("Shouldn't update static map, current edge id %s", self.current_edge_id)
        return False

    def init_static_map(self):
        init_static_map = MapState()
        init_static_map.in_junction = True
        init_static_map.target_lane_index = -1

        return init_static_map

    def update_static_map(self):

        rospy.logdebug("Updating static map")
        self.static_local_map = self.init_static_map() ## Return this one
        self.update_lane_list()
        self.update_target_lane()

        if not self.static_local_map.in_junction:
            self.calibrate_lane_index() # make the righest lane index is 0

        rospy.loginfo("Updated static map info: lane_number=%d, in_junction=%d, current_edge_id=%s, target_lane_index = %s",
                                                                            len(self.static_local_map.lanes),
                                                                            int(self.static_local_map.in_junction),
                                                                            self.current_edge_id,
                                                                            self.static_local_map.target_lane_index)

    def update_lane_list(self):
        
        map_x, map_y = self.convert_to_map_XY(self.ego_vehicle_location_x, self.ego_vehicle_location_y)

        lanes = self.hdmap.getNeighboringLanes(map_x, map_y, self.radius_lane,includeJunctions=False)
        if len(lanes) > 0:
            self.static_local_map.in_junction = False
            distancesAndLanes = sorted([(dist, lane) for lane, dist in lanes])
            dist, closestLane = distancesAndLanes[0]
            self.new_lane = closestLane
            self.new_edge = closestLane.getEdge()

            self.current_edge_id = self.new_edge.getID()
            lanes_in_edge = self.new_edge.getLanes()
            for lane in lanes_in_edge:
                connections_outgoing = lane.getOutgoing()
                # Remove fake lane
                if len(connections_outgoing) < 1:
                    continue
                add_lane = self.wrap_lane(lane)
                self.static_local_map.lanes.append(add_lane)

        

    def wrap_lane(self,lane):
        
        add_lane = LaneState()
        add_lane.index = lane.getIndex()
        shape = lane.getShape()
        central_path = Path()
        for wp in shape:
            pose = PoseStamped()
            x,y = self.convert_to_origin_XY(wp[0], wp[1])
            pose.pose.position.x = x
            pose.pose.position.y = y
            central_path.poses.append(pose)
        add_lane.central_path = central_path

        return add_lane

    def update_target_lane(self):

        while self.reference_lane_list and self.reference_lane_list[0].getEdge().getID() != self.current_edge_id:
            rospy.logdebug("Delete a passed edge: %s, current edge = %s",self.reference_lane_list[0].getEdge().getID(),self.current_edge_id)
            self.reference_lane_list.popleft()

        while self.reference_lane_list and self.reference_lane_list[0].getEdge().getID() == self.current_edge_id:
            rospy.logdebug("Delete lane with current edge id: %s, current edge = %s",self.reference_lane_list[0].getEdge().getID(),self.current_edge_id)
            self.reference_lane_list.popleft()

        if self.reference_lane_list:
            target_lane_id = self.reference_lane_list[0].getID()
            rospy.logdebug("detected next lane id: %s",target_lane_id)

            lanes_in_edge = self.new_edge.getLanes()
            for lane in lanes_in_edge:
                connections_outgoing = lane.getOutgoing()
                for connection in connections_outgoing:
                    if connection.getToLane().getID() == target_lane_id:
                        self.static_local_map.target_lane_index = lane.getIndex()
                        rospy.logdebug("Finded next target lane id = %s", self.static_local_map.target_lane_index)
                        return
        
        # TODO: reference path match target lane

        rospy.logdebug("cannot find next target lane")

    def calibrate_lane_index(self):

        first_index = self.static_local_map.lanes[0].index

        for lane in self.static_local_map.lanes:
            lane.index = lane.index - first_index

        if self.static_local_map.target_lane_index >= 0:
            self.static_local_map.target_lane_index = self.static_local_map.target_lane_index - first_index
