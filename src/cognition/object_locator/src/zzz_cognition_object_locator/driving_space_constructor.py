
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
        self._lanes_boundary_markerarray = None
        
        self._lane_dist_thres = lane_dist_thres

        self._ego_vehicle_distance_to_lane_head = [] # distance from vehicle to lane start
        self._ego_vehicle_distance_to_lane_tail = [] # distance from vehicle to lane end

        self._help_flag = 0

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
            #jxy: the converted objects are in the RoadObstacle() format

    def receive_ego_state(self, state):
        assert type(state) == RigidBodyStateStamped
        self._ego_vehicle_state = state
        #TODO: ego state should also be converted. 

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

        for i in range(len(self._surrounding_object_list)):
            rospy.loginfo("after locating: obs[%d].lane_index=%f", i, self._surrounding_object_list[i].lane_index)
            rospy.loginfo("obstacle position x:%f", self._surrounding_object_list[i].state.pose.pose.position.x)
        #jxy: why i cannot modify this property?

        #visualization
        #1. lanes
        self._lanes_markerarray = MarkerArray()

        biggest_id = 0 #TODO: better way to find the smallest id
        count = 0
        for lane in self._static_map.lanes:
            if lane.index > biggest_id:
                biggest_id = lane.index
            tempmarker = Marker() #jxy: must be put inside since it is python
            tempmarker.header.frame_id = "map"
            tempmarker.header.stamp = rospy.Time.now()
            tempmarker.ns = "zzz/cognition"
            tempmarker.id = count
            tempmarker.type = Marker.LINE_STRIP
            tempmarker.action = Marker.ADD
            tempmarker.scale.x = 0.12
            tempmarker.color.r = 1.0
            tempmarker.color.g = 0.0
            tempmarker.color.b = 0.0
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

        #2. lane boundary line
        if not self._static_map.in_junction:
            #does not draw lane when ego vehicle is in the junction
            self._lanes_boundary_markerarray = MarkerArray()

            count = 0
            for lane in self._static_map.lanes:
                tempmarker = Marker() #jxy: must be put inside since it is python
                tempmarker.header.frame_id = "map"
                tempmarker.header.stamp = rospy.Time.now()
                tempmarker.ns = "zzz/cognition"
                tempmarker.id = count

                #each lane has the right boundary, only the lane with the smallest id has the left boundary
                tempmarker.type = Marker.LINE_STRIP
                tempmarker.action = Marker.ADD
                tempmarker.scale.x = 0.12
                
                if lane.right_boundaries[0].boundary_type == 1: #broken lane is set gray
                    tempmarker.color.r = 0.6
                    tempmarker.color.g = 0.6
                    tempmarker.color.b = 0.5
                    tempmarker.color.a = 0.5
                else:
                    tempmarker.color.r = 1.0
                    tempmarker.color.g = 1.0
                    tempmarker.color.b = 1.0
                    tempmarker.color.a = 0.5
                tempmarker.lifetime = rospy.Duration(0.5)

                for lb in lane.right_boundaries:
                    p = Point()
                    p.x = lb.boundary_point.position.x
                    p.y = lb.boundary_point.position.y
                    p.z = lb.boundary_point.position.z
                    tempmarker.points.append(p)
                self._lanes_boundary_markerarray.markers.append(tempmarker)
                count = count + 1

                #biggest id: draw left lane
                if lane.index == biggest_id:
                
                    print "draw left lane boundary for the biggest id"
                    tempmarker = Marker() #jxy: must be put inside since it is python
                    tempmarker.header.frame_id = "map"
                    tempmarker.header.stamp = rospy.Time.now()
                    tempmarker.ns = "zzz/cognition"
                    tempmarker.id = count

                    #each lane has the right boundary, only the lane with the biggest id has the left boundary
                    tempmarker.type = Marker.LINE_STRIP
                    tempmarker.action = Marker.ADD
                    tempmarker.scale.x = 0.12
                    if lane.left_boundaries[0].boundary_type == 1: #broken lane is set gray
                        tempmarker.color.r = 0.6
                        tempmarker.color.g = 0.6
                        tempmarker.color.b = 0.6
                        tempmarker.color.a = 0.5
                    else:
                        tempmarker.color.r = 1.0
                        tempmarker.color.g = 1.0
                        tempmarker.color.b = 1.0
                        tempmarker.color.a = 0.5
                    tempmarker.lifetime = rospy.Duration(0.5)

                    for lb in lane.left_boundaries:
                        p = Point()
                        p.x = lb.boundary_point.position.x
                        p.y = lb.boundary_point.position.y
                        p.z = lb.boundary_point.position.z
                        tempmarker.points.append(p)
                    self._lanes_boundary_markerarray.markers.append(tempmarker)
                    count = count + 1

        #3. obstacle
        self._obstacles_markerarray = MarkerArray()
        
        count = 0
        if self._surrounding_object_list is not None:
            for obs in self._surrounding_object_list:
                dist_to_ego = math.sqrt(math.pow((obs.state.pose.pose.position.x - self._ego_vehicle_state.state.pose.pose.position.x),2) 
                    + math.pow((obs.state.pose.pose.position.y - self._ego_vehicle_state.state.pose.pose.position.y),2))
                
                if dist_to_ego < 50:
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
                    if obs.lane_index == -1:
                        tempmarker.color.r = 1.0
                        tempmarker.color.g = 1.0
                        tempmarker.color.b = 1.0
                    elif obs.lane_dist_left_t == 0 or obs.lane_dist_right_t == 0:
                        # those who is on the lane boundary, warn by yellow
                        tempmarker.color.r = 1.0
                        tempmarker.color.g = 1.0
                        tempmarker.color.b = 0.0
                    else:
                        tempmarker.color.r = 1.0
                        tempmarker.color.g = 0.0
                        tempmarker.color.b = 1.0
                    if self._static_map.in_junction:
                        tempmarker.color.r = 1.0
                        tempmarker.color.g = 0.0
                        tempmarker.color.b = 1.0
                    tempmarker.color.a = 0.5
                    tempmarker.lifetime = rospy.Duration(0.5)

                    self._obstacles_markerarray.markers.append(tempmarker)
                    count = count + 1
                    
            for obs in self._surrounding_object_list:
                dist_to_ego = math.sqrt(math.pow((obs.state.pose.pose.position.x - self._ego_vehicle_state.state.pose.pose.position.x),2) 
                    + math.pow((obs.state.pose.pose.position.y - self._ego_vehicle_state.state.pose.pose.position.y),2))
                
                if dist_to_ego < 50:
                    tempmarker = Marker() #jxy: must be put inside since it is python
                    tempmarker.header.frame_id = "map"
                    tempmarker.header.stamp = rospy.Time.now()
                    tempmarker.ns = "zzz/cognition"
                    tempmarker.id = count
                    tempmarker.type = Marker.TEXT_VIEW_FACING
                    tempmarker.action = Marker.ADD
                    hahaha = obs.state.pose.pose.position.z + 1.0
                    tempmarker.pose.position.x = obs.state.pose.pose.position.x
                    tempmarker.pose.position.y = obs.state.pose.pose.position.y
                    tempmarker.pose.position.z = hahaha
                    tempmarker.scale.z = 0.6
                    tempmarker.color.r = 1.0
                    tempmarker.color.g = 0.0
                    tempmarker.color.b = 1.0
                    tempmarker.color.a = 0.5
                    tempmarker.text = "help flag: " + str(self._help_flag) + "\n in junction: " + str(self._static_map.in_junction) + "\n lane_num: " + \
                        str(len(self._static_map.lanes)) + " lane_index: " + str(obs.lane_index) + "\n lane_dist_right_t: " + str(obs.lane_dist_right_t) + "\n lane_dist_left_t: " + str(obs.lane_dist_left_t)
                    tempmarker.lifetime = rospy.Duration(0.5)

                    self._obstacles_markerarray.markers.append(tempmarker)
                    count = count + 1
            
            for obs in self._surrounding_object_list:
                dist_to_ego = math.sqrt(math.pow((obs.state.pose.pose.position.x - self._ego_vehicle_state.state.pose.pose.position.x),2) 
                    + math.pow((obs.state.pose.pose.position.y - self._ego_vehicle_state.state.pose.pose.position.y),2))
                
                if dist_to_ego < 50:
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

        #4. ego vehicle visualization
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
        tempmarker.scale.z = 1.8
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

        #5. traffic lights
        self._traffic_lights_markerarray = MarkerArray()

        #TODO: now no lights are in. I'll check it when I run the codes.
        
        #lights = self._traffic_light_detection.detections
        #rospy.loginfo("lights num: %d\n\n", len(lights))
        
        rospy.logdebug("Updated driving space")

    # ========= For in lane =========

    def locate_object_in_lane(self, object, dimension, dist_list=None):
        '''
        Calculate (continuous) lane index for a object.
        Parameters: dist_list is the distance buffer. If not provided, it will be calculated
        '''
        simplify_flag = 0

        if not dist_list:
            dist_list = np.array([dist_from_point_to_polyline2d(
                object.pose.pose.position.x,
                object.pose.pose.position.y,
                lane) for lane in self._static_map_lane_path_array]) # here lane is a python list of (x, y)
        
        # Check if there's only two lanes
        if len(self._static_map.lanes) < 2:
            closest_lane = second_closest_lane = 0
        else:
            closest_lane, second_closest_lane = np.abs(dist_list[:, 0]).argsort()[:2]

        # Signed distance from target to two closest lane
        closest_lane_dist, second_closest_lane_dist = dist_list[closest_lane, 0], dist_list[second_closest_lane, 0]

        if abs(closest_lane_dist) > self._lane_dist_thres:
            return -1, -99, -99, -99, -99 # TODO: return reasonable value

        lane = self._static_map.lanes[closest_lane]
        left_boundary_array = np.array([(lbp.boundary_point.position.x, lbp.boundary_point.position.y) for lbp in lane.left_boundaries])
        right_boundary_array = np.array([(lbp.boundary_point.position.x, lbp.boundary_point.position.y) for lbp in lane.right_boundaries])

        # Distance to lane considering the size of the object
        x = object.pose.pose.orientation.x
        y = object.pose.pose.orientation.y
        z = object.pose.pose.orientation.z
        w = object.pose.pose.orientation.w

        rotation_mat = np.array([[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x], [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]])
        rotation_mat_inverse = np.linalg.inv(rotation_mat) #those are the correct way to deal with quaternion

        vector_x = np.array([dimension.length_x, 0, 0])
        vector_y = np.array([0, dimension.length_y, 0])
        dx = np.matmul(rotation_mat_inverse, vector_x)
        dy = np.matmul(rotation_mat_inverse, vector_y)

        #the four corners of the object, in bird view: left front is 0, counterclockwise.
        #TODO: may consider 8 corners in the future
        corner_list_x = np.zeros(4)
        corner_list_y = np.zeros(4)
        corner_list_x[0] = object.pose.pose.position.x + dx[0]/2.0 + dy[0]/2.0
        corner_list_y[0] = object.pose.pose.position.y + dx[1]/2.0 + dy[1]/2.0
        corner_list_x[1] = object.pose.pose.position.x - dx[0]/2.0 + dy[0]/2.0
        corner_list_y[1] = object.pose.pose.position.y - dx[1]/2.0 + dy[1]/2.0
        corner_list_x[2] = object.pose.pose.position.x - dx[0]/2.0 - dy[0]/2.0
        corner_list_y[2] = object.pose.pose.position.y - dx[1]/2.0 - dy[1]/2.0
        corner_list_x[3] = object.pose.pose.position.x + dx[0]/2.0 - dy[0]/2.0
        corner_list_y[3] = object.pose.pose.position.y + dx[1]/2.0 - dy[1]/2.0

        dist_left_list_all = np.array([dist_from_point_to_polyline2d(
                corner_list_x[i],
                corner_list_y[i],
                left_boundary_array) for i in range(4)])
        dist_right_list_all = np.array([dist_from_point_to_polyline2d(
                corner_list_x[i],
                corner_list_y[i],
                right_boundary_array) for i in range(4)])

        dist_left_list = dist_left_list_all[:, 0]
        dist_right_list = dist_right_list_all[:, 0]

        lane_dist_left_t = -99
        lane_dist_right_t = -99

        rospy.loginfo("dist to lane center line: %f", abs(closest_lane_dist))
        rospy.loginfo("position x: %f, y: %f", object.pose.pose.position.x, object.pose.pose.position.y)
        rospy.loginfo("dimension x: %f, y: %f", dimension.length_x, dimension.length_y)
        '''
        print dx
        print dy
        
        print corner_list_x
        print corner_list_y
        print dist_left_list_all
        print dist_right_list_all
        '''
        if np.min(dist_left_list) * np.max(dist_left_list) <= 0:
            rospy.loginfo("the object is on the left boundary of lane %d\n", closest_lane)
            lane_dist_left_t = 0
        else:
            lane_dist_left_t = np.sign(np.min(dist_left_list)) * np.min(np.abs(dist_left_list))
            print lane_dist_left_t

        if np.min(dist_right_list) * np.max(dist_right_list) <= 0:
            rospy.loginfo("the object is on the right boundary of lane %d\n", closest_lane)
            lane_dist_right_t = 0
        else:
            lane_dist_right_t = np.sign(np.min(dist_right_list)) * np.min(np.abs(dist_right_list))

        if np.min(dist_left_list) * np.max(dist_left_list) > 0 and np.min(dist_right_list) * np.max(dist_right_list) > 0:
            rospy.loginfo("dist to left: %f, dist to right: %f", lane_dist_left_t, lane_dist_right_t)
            if np.min(dist_left_list) * np.max(dist_right_list) < 0:
                rospy.loginfo("the object is between the boundaries of lane %d\n", closest_lane)
                rospy.loginfo("length x: %f, length_y: %f", dimension.length_x, dimension.length_y)

            else:
                rospy.loginfo("the object is out of the road")
                closest_lane = -1

        #simplify: not considering the box size
        if simplify_flag == 1:
            lane_dist_left_t = dist_from_point_to_polyline2d(object.pose.pose.position.x, object.pose.pose.position.y,
                    left_boundary_array)
            lane_dist_right_t = dist_from_point_to_polyline2d(object.pose.pose.position.x, object.pose.pose.position.y,
                    right_boundary_array)

            print "center position:"
            print object.pose.pose.position.x
            print object.pose.pose.position.y
            print "center dist:"
            print lane_dist_left_t
            print lane_dist_right_t
            
            print left_boundary_array

            lane_dist_left_t = dist_from_point_to_polyline2d(object.pose.pose.position.x + dx[0]/2.0 + dy[0]/2.0, object.pose.pose.position.y + dx[1]/2.0 + dy[1]/2.0,
                    left_boundary_array)
            lane_dist_right_t = dist_from_point_to_polyline2d(object.pose.pose.position.x + dx[0]/2.0 + dy[0]/2.0, object.pose.pose.position.y + dx[1]/2.0 + dy[1]/2.0,
                    right_boundary_array)
            
        ffstate = get_frenet_state(object,
                        self._static_map_lane_path_array[closest_lane],
                        self._static_map_lane_tangets[closest_lane]
                    )
        lane_anglediff = ffstate.psi
        lane_dist_s = ffstate.s # this is also helpful in getting ego s coordinate in the road

        

        # Judge whether the point is outside of lanes
        if closest_lane == -1:
            # The object is at left or right most
            return closest_lane, lane_dist_left_t, lane_dist_right_t, lane_anglediff, lane_dist_s
        else:
            # The object is between center line of lanes
            a, b = closest_lane, second_closest_lane
            la, lb = abs(closest_lane_dist), abs(second_closest_lane_dist)
            return (b*la + a*lb)/(lb + la), lane_dist_left_t, lane_dist_right_t, lane_anglediff, lane_dist_s
        

    def locate_obstacle_in_lanes(self):
        if self._surrounding_object_list == None:
            return
        for i in range(len(self._surrounding_object_list)):
            if len(self._static_map.lanes) != 0:
                self._surrounding_object_list[i].lane_index, self._surrounding_object_list[i].lane_dist_left_t, self._surrounding_object_list[i].lane_dist_right_t, \
                    self._surrounding_object_list[i].lane_anglediff, self._surrounding_object_list[i].lane_dist_s = self.locate_object_in_lane(self._surrounding_object_list[i].state, self._surrounding_object_list[i].dimension)
            else:
                self._surrounding_object_list[i].lane_index = -1
            rospy.loginfo("before locating: obj[%d].lane_index=%f", i, self._surrounding_object_list[i].lane_index)
            self._surrounding_object_list[i].state.pose.pose.position.x = 0
            rospy.loginfo("obstacle position x:%f", self._surrounding_object_list[i].state.pose.pose.position.x)

    def locate_ego_vehicle_in_lanes(self, lane_end_dist_thres=2, lane_dist_thres=5):
        if self._static_map_lane_path_array == None: # TODO: This should not happen 
            return

        dist_list = np.array([dist_from_point_to_polyline2d(
            self._ego_vehicle_state.state.pose.pose.position.x, self._ego_vehicle_state.state.pose.pose.position.y,
            lane, return_end_distance=True)
            for lane in self._static_map_lane_path_array])
        ego_dimension = DimensionWithCovariance()
        ego_dimension.length_x = 4.0
        ego_dimension.length_y = 2.0 #jxy: I don't know
        ego_dimension.length_z = 1.8
        ego_lane_index, _, _, _, _ = self.locate_object_in_lane(self._ego_vehicle_state.state, ego_dimension)
        #TODO: should be added to converted ego msg

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
        #jxy: demanding that the lights are in the same order as the lanes.

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
