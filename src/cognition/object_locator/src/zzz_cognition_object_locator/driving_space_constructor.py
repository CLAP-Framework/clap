
import rospy
import numpy as np
import math

from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_navigation_msgs.msg import Map, Lane
from zzz_navigation_msgs.utils import get_lane_array
from zzz_cognition_msgs.msg import MapState, LaneState, RoadObstacle
from zzz_cognition_msgs.utils import convert_tracking_box, default_msg as cognition_default
from zzz_perception_msgs.msg import TrackingBoxArray, DetectionBoxArray, ObjectSignals, DimensionWithCovariance
from zzz_common.geometry import dist_from_point_to_polyline2d, wrap_angle
from zzz_common.kinematics import get_frenet_state
from zzz_cognition_msgs.msg import DrivingSpace
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from zzz_driver_msgs.utils import get_speed, get_yaw

#jxy 20191125: first output the driving space, then use the driving space for cognition. 
#For this demo version, it will be a unified module, in future versions, this will be split into 2 modules.

class DrivingSpaceConstructor:
    def __init__(self, lane_dist_thres=5):
        self._static_map = Map()
        self._static_map.in_junction = True
        self._static_map_lane_path_array = None
        self._static_map_lane_tangets = None
        self._driving_space = None
        self._surrounding_object_list = None
        self._ego_vehicle_state = None
        self._traffic_light_detection = None
        self._obstacles_markerarray = None
        
        self._lane_dist_thres = lane_dist_thres

        self._ego_vehicle_distance_to_lane_head = [] # distance from vehicle to lane start
        self._ego_vehicle_distance_to_lane_tail = [] # distance from vehicle to lane end

    @property
    def driving_space(self):
        return self._driving_space

    def obstacles_markerarray(self):
        return self._obstacles_markerarray

    # ====== Data Receiver =======

    def receive_static_map(self, static_map):
        assert type(static_map) == Map

        self._static_map = static_map
        self._static_map_lane_path_array = get_lane_array(static_map.lanes)
        self._static_map_lane_tangets = [[point.tangent for point in lane.central_path_points] for lane in static_map.lanes]
        rospy.logdebug("Updated Local Static Map: lanes_num = %d, in_junction = %d, target_lane_index = %d",
            len(self._static_map.lanes), int(self._static_map.in_junction), self._static_map.target_lane_index)

    def receive_object_list(self, object_list):
        assert type(object_list) == TrackingBoxArray
        if self._ego_vehicle_state != None:
            self._surrounding_object_list = convert_tracking_box(object_list, self._ego_vehicle_state)

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped
        self._ego_vehicle_state = state

    def receive_traffic_light_detection(self, detection):
        assert type(detection) == DetectionBoxArray
        self._traffic_light_detection = detection

    # ====== Data Updator =======

    def update_driving_space(self):
        self._driving_space = DrivingSpace()

        self._driving_space.header.frame_id = "map"
        self._driving_space.header.stamp = rospy.Time.now()

        self._driving_space.ego_state = self._ego_vehicle_state.state

        #TODO: drivable area. It should be updated by obstacles. Only static drivable area is not OK.

        self._driving_space.obstacles = self._surrounding_object_list

        rospy.logdebug("len(self._static_map.lanes): %d", len(self._static_map.lanes))

        if self._static_map.in_junction or len(self._static_map.lanes) == 0:
            rospy.logdebug("In junction due to static map report junction location")
        else:
            for lane in self._static_map.lanes:
                self._driving_space.lanes.append(lane)
            #jxy: why is target lane in static map?
            self.locate_ego_vehicle_in_lanes()
            self.locate_obstacle_in_lanes()
            self.locate_stop_sign_in_lanes()
            self.locate_speed_limit_in_lanes()

        #visualization
        self._lanes_markerarray = MarkerArray()

        count = 0
        for lane in self._static_map.lanes:
            tempmarker = Marker() #jxy: must be put inside since it is python
            tempmarker.header.frame_id = "map"
            tempmarker.header.stamp = rospy.Time.now()
            tempmarker.ns = "zzz/cognition"
            tempmarker.id = count
            tempmarker.type = Marker.LINE_STRIP
            tempmarker.action = Marker.ADD
            tempmarker.scale.x = 0.12
            tempmarker.color.r = 1.0
            tempmarker.color.g = 1.0
            tempmarker.color.b = 1.0
            tempmarker.color.a = 0.5
            tempmarker.lifetime = rospy.Duration(0.5)

            for lanepoint in lane.central_path_points:
                p = Point()
                p.x = lanepoint.position.x
                p.y = lanepoint.position.y
                p.z = lanepoint.position.z
                tempmarker.points.append(p)
            self._lanes_markerarray.markers.append(tempmarker)
            count = count + 1


        self._obstacles_markerarray = MarkerArray()
        
        count = 0
        if self._surrounding_object_list is not None:
            for obs in self._surrounding_object_list:
                if math.sqrt(math.pow((obs.state.pose.pose.position.x - self._ego_vehicle_state.state.pose.pose.position.x),2) + math.pow((obs.state.pose.pose.position.y - self._ego_vehicle_state.state.pose.pose.position.y),2)) < 50:
                    tempmarker = Marker() #jxy: must be put inside since it is python
                    tempmarker.header.frame_id = "map"
                    tempmarker.header.stamp = rospy.Time.now()
                    tempmarker.ns = "zzz/cognition"
                    tempmarker.id = count
                    tempmarker.type = Marker.CUBE
                    tempmarker.action = Marker.ADD
                    tempmarker.pose = obs.state.pose.pose
                    tempmarker.scale.x = obs.dimension.length_x
                    tempmarker.scale.y = obs.dimension.length_y
                    tempmarker.scale.z = obs.dimension.length_z
                    tempmarker.color.r = 1.0
                    tempmarker.color.g = 0.0
                    tempmarker.color.b = 1.0
                    tempmarker.color.a = 0.5
                    tempmarker.lifetime = rospy.Duration(0.5)

                    self._obstacles_markerarray.markers.append(tempmarker)
                    count = count + 1
            
            for obs in self._surrounding_object_list:
                if math.sqrt(math.pow((obs.state.pose.pose.position.x - self._ego_vehicle_state.state.pose.pose.position.x),2) + math.pow((obs.state.pose.pose.position.y - self._ego_vehicle_state.state.pose.pose.position.y),2)) < 50:
                    tempmarker = Marker() #jxy: must be put inside since it is python
                    tempmarker.header.frame_id = "map"
                    tempmarker.header.stamp = rospy.Time.now()
                    tempmarker.ns = "zzz/cognition"
                    tempmarker.id = count
                    tempmarker.type = Marker.ARROW
                    tempmarker.action = Marker.ADD
                    tempmarker.scale.x = 0.4
                    tempmarker.scale.y = 0.7
                    tempmarker.scale.z = 0.75
                    tempmarker.color.r = 1.0
                    tempmarker.color.g = 1.0
                    tempmarker.color.b = 0.0
                    tempmarker.color.a = 0.5
                    tempmarker.lifetime = rospy.Duration(0.5)

                    startpoint = Point()
                    endpoint = Point()
                    startpoint.x = obs.state.pose.pose.position.x
                    startpoint.y = obs.state.pose.pose.position.y
                    startpoint.z = obs.state.pose.pose.position.z
                    endpoint.x = obs.state.pose.pose.position.x + obs.state.twist.twist.linear.x
                    endpoint.y = obs.state.pose.pose.position.y + obs.state.twist.twist.linear.y
                    endpoint.z = obs.state.pose.pose.position.z + obs.state.twist.twist.linear.z
                    tempmarker.points.append(startpoint)
                    tempmarker.points.append(endpoint)

                    self._obstacles_markerarray.markers.append(tempmarker)
                    count = count + 1

        #ego vehicle visualization
        self._ego_markerarray = MarkerArray()

        tempmarker = Marker()
        tempmarker.header.frame_id = "map"
        tempmarker.header.stamp = rospy.Time.now()
        tempmarker.ns = "zzz/cognition"
        tempmarker.id = 1
        tempmarker.type = Marker.CUBE
        tempmarker.action = Marker.ADD
        tempmarker.pose = self._ego_vehicle_state.state.pose.pose
        tempmarker.scale.x = 4.0 #jxy: I don't know...
        tempmarker.scale.y = 2.0
        tempmarker.scale.z = 1.0
        tempmarker.color.r = 1.0
        tempmarker.color.g = 0.0
        tempmarker.color.b = 0.0
        tempmarker.color.a = 0.5
        tempmarker.lifetime = rospy.Duration(0.5)

        self._ego_markerarray.markers.append(tempmarker)

        #quaternion transform for ego velocity

        x = self._ego_vehicle_state.state.pose.pose.orientation.x
        y = self._ego_vehicle_state.state.pose.pose.orientation.y
        z = self._ego_vehicle_state.state.pose.pose.orientation.z
        w = self._ego_vehicle_state.state.pose.pose.orientation.w

        rotation_mat = np.array([[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x], [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]])
        rotation_mat_inverse = np.linalg.inv(rotation_mat) #those are the correct way to deal with quaternion

        vel_self = np.array([[self._ego_vehicle_state.state.twist.twist.linear.x], [self._ego_vehicle_state.state.twist.twist.linear.y], [self._ego_vehicle_state.state.twist.twist.linear.z]])
        vel_world = np.matmul(rotation_mat_inverse, vel_self)
        #check if it should be reversed
        ego_vx_world = vel_world[0]
        ego_vy_world = vel_world[1]
        ego_vz_world = vel_world[2]

        tempmarker = Marker()
        tempmarker.header.frame_id = "map"
        tempmarker.header.stamp = rospy.Time.now()
        tempmarker.ns = "zzz/cognition"
        tempmarker.id = 2
        tempmarker.type = Marker.ARROW
        tempmarker.action = Marker.ADD
        tempmarker.scale.x = 0.4
        tempmarker.scale.y = 0.7
        tempmarker.scale.z = 0.75
        tempmarker.color.r = 1.0
        tempmarker.color.g = 1.0
        tempmarker.color.b = 0.0
        tempmarker.color.a = 0.5
        tempmarker.lifetime = rospy.Duration(0.5)

        startpoint = Point()
        endpoint = Point()
        startpoint.x = self._ego_vehicle_state.state.pose.pose.position.x
        startpoint.y = self._ego_vehicle_state.state.pose.pose.position.y
        startpoint.z = self._ego_vehicle_state.state.pose.pose.position.z
        endpoint.x = self._ego_vehicle_state.state.pose.pose.position.x + ego_vx_world
        endpoint.y = self._ego_vehicle_state.state.pose.pose.position.y + ego_vy_world
        endpoint.z = self._ego_vehicle_state.state.pose.pose.position.z + ego_vz_world
        tempmarker.points.append(startpoint)
        tempmarker.points.append(endpoint)

        self._ego_markerarray.markers.append(tempmarker)
        
        rospy.logdebug("Updated driving space")

    # ========= For in lane =========

    def locate_object_in_lane(self, object_state, dist_list=None):
        '''
        Calculate (continuous) lane index for a object.
        Parameters: dist_list is the distance buffer. If not provided, it will be calculated
        '''

        #jxy: See how important lane boundary is. If there are lane boundaries, the location process will be much easier.

        if not dist_list:
            dist_list = np.array([dist_from_point_to_polyline2d(
                object_state.pose.pose.position.x,
                object_state.pose.pose.position.y,
                lane) for lane in self._static_map_lane_path_array])
        
        # Check if there's only two lanes
        if len(self._static_map.lanes) < 2:
            closest_lane = second_closest_lane = 0
        else:
            closest_lane, second_closest_lane = np.abs(dist_list[:, 0]).argsort()[:2]

        # Signed distance from target to two closest lane
        closest_lane_dist, second_closest_lane_dist = dist_list[closest_lane, 0], dist_list[second_closest_lane, 0]

        if abs(closest_lane_dist) > self._lane_dist_thres:
            return -1 # TODO: return reasonable value

        # Judge whether the point is outside of lanes
        if closest_lane == second_closest_lane or closest_lane_dist * second_closest_lane_dist > 0:
            #jxy: why can the closest and the second closest be the same?
            # The object is at left or right most
            return closest_lane
        else:
            # The object is between center line of lanes
            a, b = closest_lane, second_closest_lane
            la, lb = abs(closest_lane_dist), abs(second_closest_lane_dist)
            return (b*la + a*lb)/(lb + la)
            #jxy: what is it? If I drive zigzag along the lane boundary, it will regard my behavior as frequent jumping between two lane centerlines

    def locate_obstacle_in_lanes(self):
        if self._surrounding_object_list == None:
            return
        for obj in self._surrounding_object_list:
            if len(self._static_map.lanes) != 0:
                obj.lane_index = self.locate_object_in_lane(obj.state)
            else:
                obj.lane_index = -1

    def locate_ego_vehicle_in_lanes(self, lane_end_dist_thres=2, lane_dist_thres=5):
        if self._static_map_lane_path_array == None: # TODO: This should not happen 
            return

        dist_list = np.array([dist_from_point_to_polyline2d(
            self._ego_vehicle_state.state.pose.pose.position.x, self._ego_vehicle_state.state.pose.pose.position.y,
            lane, return_end_distance=True)
            for lane in self._static_map_lane_path_array])  
        ego_lane_index = self.locate_object_in_lane(self._ego_vehicle_state.state)

        self._ego_vehicle_distance_to_lane_head = dist_list[:, 3]
        self._ego_vehicle_distance_to_lane_tail = dist_list[:, 4]
        if ego_lane_index < 0 or self._ego_vehicle_distance_to_lane_tail[int(ego_lane_index)] <= lane_end_dist_thres:
            # Drive into junction, wait until next map
            rospy.logdebug("In junction due to close to intersection")
            return
        else:
            self._driving_space.ego_lane_index = ego_lane_index

    def locate_traffic_light_in_lanes(self):
        # TODO: Currently it's a very simple rule to locate the traffic lights
        if self._traffic_light_detection is None:
            return
        lights = self._traffic_light_detection.detections

        total_lane_num = len(self._static_map.lanes)
        if len(lights) == 1:
            for i in range(total_lane_num):
                if lights[0].signal == ObjectSignals.TRAFFIC_LIGHT_RED:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                elif lights[0].signal == ObjectSignals.TRAFFIC_LIGHT_YELLOW:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_YIELD
                elif lights[0].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        elif len(lights) > 1 and len(lights) == total_lane_num:
            for i in range(total_lane_num):
                if lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_RED:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                elif lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_YELLOW:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_YIELD
                elif lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    self._static_map.mmap.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        elif len(lights) > 1 and len(lights) != total_lane_num:
            red = True
            for i in range(len(lights)):
                if lights[i].signal == ObjectSignals.TRAFFIC_LIGHT_GREEN:
                    red = False
            for i in range(total_lane_num):
                if red:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_STOP
                else:
                    self._static_map.lanes[i].map_lane.stop_state = Lane.STOP_STATE_THRU
        
    def locate_stop_sign_in_lanes(self):
        '''
        Put stop sign detections into lanes
        '''
        # TODO: Implement this
        pass

    def locate_speed_limit_in_lanes(self):
        '''
        Put stop sign detections into lanes
        '''
        # TODO(zhcao): Change the speed limit according to the map or the traffic sign(perception)
        # Now we set the multilane speed limit as 40 km/h.
        total_lane_num = len(self._static_map.lanes)
        for i in range(total_lane_num):
            self._static_map.lanes[i].speed_limit = 40
