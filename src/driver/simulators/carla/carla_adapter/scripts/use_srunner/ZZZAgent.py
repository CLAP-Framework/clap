# This script is for running Carla scenario_runner

import carla
import math
import numpy
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
import rosgraph

from srunner.challenge.autoagents.autonomous_agent import AutonomousAgent, Track
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel, CarlaEgoVehicleControl, CarlaWorldInfo
from zzz_perception_msgs.msg import TrackingBoxArray, TrackingBox, ObjectClass

from sensor_msgs.point_cloud2 import create_cloud_xyz32
import tf
from cv_bridge import CvBridge
import os
import subprocess
import signal
import time

class ZZZAgent(AutonomousAgent):
    """
    Base class for ROS-based stacks.

    Derive from it and implement the sensors() method.

    Please define TEAM_CODE_ROOT in your environment.
    The stack is started by executing $TEAM_CODE_ROOT/start.sh

    The sensor data is published on similar topics as with the carla-ros-bridge. You can find details about
    the utilized datatypes there.

    This agent expects a roscore to be running.
    """

    def setup(self, path_to_conf_file):
        """
        setup agent
        """
        # XXX: You need to tweak valid_sensors_configuration in challenge_evaluator_routes.py to let object_finder sensor work
        self.track = Track.ALL_SENSORS_HDMAP_WAYPOINTS
        self.stack_thread = None

        #get start_script from environment
        team_code_path = os.environ['TEAM_CODE_ROOT']
        if not team_code_path or not os.path.exists(team_code_path):
            raise ValueError("Path '{}' defined by TEAM_CODE_ROOT invalid".format(team_code_path))
        start_script = "{}/start.sh".format(team_code_path)
        if not os.path.exists(start_script):
            raise ValueError("File '{}' defined by TEAM_CODE_ROOT invalid".format(start_script))

        #set use_sim_time via commandline before init-node
        process = subprocess.Popen("rosparam set use_sim_time true", shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        process.wait()
        if process.returncode:
            raise RuntimeError("Could not set use_sim_time")

        #initialize ros node
        rospy.init_node('ros_agent', anonymous=True)

        #publish first clock value '0'
        self.clock_publisher = rospy.Publisher('clock', Clock, queue_size=10, latch=True)
        self.clock_publisher.publish(Clock(rospy.Time.from_sec(0)))

        #execute script that starts the ad stack (remains running)
        rospy.loginfo("Executing stack...")
        self.stack_process = subprocess.Popen(start_script, shell=True, preexec_fn=os.setpgrp)

        self.use_stepping_mode = True
        self.vehicle_control_event = threading.Event()
        self.timestamp = None
        self.speed = 0
        self.global_plan_published = False

        self.vehicle_info_publisher = None
        self.vehicle_status_publisher = None
        self.odometry_publisher = None
        self.world_info_publisher = None
        self.map_file_publisher = None
        self.objects_publisher = None
        self.objects_storage = {}
        self.current_map_name = None
        self.tf_broadcaster = None
        self.step_mode_possible = False

        self.vehicle_control_subscriber = rospy.Subscriber('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, self.on_vehicle_control)

        self.current_control = carla.VehicleControl()

        self.global_plan_calibrated = False
        self.lon_origin = 0
        self.lat_origin = 0
        self.waypoint_publisher = rospy.Publisher(
            '/carla/ego_vehicle/waypoints', Path, queue_size=1, latch=True)

        self.publisher_map = {}
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()

        #setup ros publishers for sensors
        for sensor in self.sensors():
            self.id_to_sensor_type_map[sensor['id']] = sensor['type']
            if sensor['type'] == 'sensor.camera.rgb':
                self.publisher_map[sensor['id']] = rospy.Publisher('/carla/ego_vehicle/camera/rgb/' + sensor['id'] + "/image_color", Image, queue_size=1, latch=True)
                self.id_to_camera_info_map[sensor['id']] = self.build_camera_info(sensor)
                self.publisher_map[sensor['id'] + '_info'] = rospy.Publisher('/carla/ego_vehicle/camera/rgb/' + sensor['id'] + "/camera_info", CameraInfo, queue_size=1, latch=True)
            elif sensor['type'] == 'sensor.lidar.ray_cast':
                self.publisher_map[sensor['id']] = rospy.Publisher('/carla/ego_vehicle/lidar/' + sensor['id'] + "/point_cloud", PointCloud2, queue_size=1, latch=True)
            elif sensor['type'] == 'sensor.other.gnss':
                self.publisher_map[sensor['id']] = rospy.Publisher('/carla/ego_vehicle/gnss/' + sensor['id'] + "/fix", NavSatFix, queue_size=1, latch=True)
            elif sensor['type'] == 'sensor.can_bus':
                if not self.vehicle_info_publisher:
                    self.vehicle_info_publisher = rospy.Publisher('/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=1, latch=True)
                if not self.vehicle_status_publisher:
                    self.vehicle_status_publisher = rospy.Publisher('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, queue_size=1, latch=True)
            elif sensor['type'] == 'sensor.hd_map':
                if not self.odometry_publisher:
                    self.odometry_publisher = rospy.Publisher('/carla/ego_vehicle/odometry', Odometry, queue_size=1, latch=True)
                if not self.world_info_publisher:
                    self.world_info_publisher = rospy.Publisher('/carla/world_info', CarlaWorldInfo, queue_size=1, latch=True)
                if not self.map_file_publisher:
                    self.map_file_publisher = rospy.Publisher('/carla/map_file', String, queue_size=1, latch=True)
                if not self.tf_broadcaster:
                    self.tf_broadcaster = tf.TransformBroadcaster()
            elif sensor['type'] == 'sensor.object_finder':
                if not self.objects_publisher:
                    self.objects_publisher = rospy.Publisher('/carla/ego_vehicle/objects', TrackingBoxArray, queue_size=1, latch=True)
            else:
                raise TypeError("Invalid sensor type: {}".format(sensor['type']))

    def destroy(self):
        if self.stack_process and self.stack_process.poll() is None:
            rospy.loginfo("Sending SIGTERM to stack...")
            os.killpg(os.getpgid(self.stack_process.pid), signal.SIGTERM)
            rospy.loginfo("Waiting for termination of stack...")
            self.stack_process.wait()
            time.sleep(5)
            rospy.loginfo("Terminated stack.")

        rospy.loginfo("Stack is no longer running")
        self.world_info_publisher.unregister()
        self.map_file_publisher.unregister()
        self.vehicle_status_publisher.unregister()
        self.vehicle_info_publisher.unregister()
        self.waypoint_publisher.unregister()
        self.objects_publisher.unregister()
        self.stack_process = None
        rospy.loginfo("Cleanup finished")

    def on_vehicle_control(self, data):
        """
        callback if a new vehicle control command is received
        """
        cmd = carla.VehicleControl()
        cmd.throttle = data.throttle
        cmd.steer = data.steer
        cmd.brake = data.brake
        cmd.hand_brake = data.hand_brake
        cmd.reverse = data.reverse
        cmd.gear = data.gear
        cmd.manual_gear_shift = data.manual_gear_shift
        self.current_control = cmd
        if not self.vehicle_control_event.is_set():
            self.vehicle_control_event.set()
        # After the first vehicle control is sent out, it is possible to use the stepping mode
        self.step_mode_possible = True

    def build_camera_info(self, attributes):
        """
        Private function to compute camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        # store info without header
        camera_info.header = None
        camera_info.width = int(attributes['width'])
        camera_info.height = int(attributes['height'])
        camera_info.distortion_model = 'plumb_bob'
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(attributes['fov']) * math.pi / 360.0))
        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        return camera_info

    def publish_plan(self):
        """
        publish the global plan
        """
        msg = Path()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        for wp in self._global_plan_world_coord:
            pose = PoseStamped()
            pose.pose.position.x = wp[0].location.x
            pose.pose.position.y = -wp[0].location.y
            pose.pose.position.z = wp[0].location.z
            quaternion = tf.transformations.quaternion_from_euler(
                0, 0, -math.radians(wp[0].rotation.yaw))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            msg.poses.append(pose)

        rospy.loginfo("Publishing Plan...")
        self.waypoint_publisher.publish(msg)

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.from_sec(self.timestamp)
        return header

    def publish_lidar(self, sensor_id, data):
        """
        Function to publish lidar data
        """
        header = self.get_header()
        header.frame_id = 'ego_vehicle/lidar/{}'.format(sensor_id)

        lidar_data = numpy.frombuffer(
            data, dtype=numpy.float32)
        lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 3), 3))
        # we take the oposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        # we need a copy here, because the data are read only in carla numpy
        # array
        lidar_data = -lidar_data
        # we also need to permute x and y
        lidar_data = lidar_data[..., [1, 0, 2]]
        msg = create_cloud_xyz32(header, lidar_data)
        self.publisher_map[sensor_id].publish(msg)

    def publish_gnss(self, sensor_id, data):
        """
        Function to publish gnss data
        """
        msg = NavSatFix()
        msg.header = self.get_header()
        msg.header.frame_id = 'gps'
        msg.latitude = data[0]
        msg.longitude = data[1]
        msg.altitude = data[2]
        msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        self.publisher_map[sensor_id].publish(msg)

    def publish_camera(self, sensor_id, data):
        """
        Function to publish camera data
        """
        msg = self.cv_bridge.cv2_to_imgmsg(data, encoding='bgra8')
        # the camera data is in respect to the camera's own frame
        msg.header = self.get_header()
        msg.header.frame_id = 'ego_vehicle/camera/rgb/{}'.format(sensor_id)

        cam_info = self.id_to_camera_info_map[sensor_id]
        cam_info.header = msg.header
        self.publisher_map[sensor_id + '_info'].publish(cam_info)
        self.publisher_map[sensor_id].publish(msg)

    def publish_can(self, sensor_id, data):
        """
        publish can data
        """
        if not self.vehicle_info_publisher:
            self.vehicle_info_publisher = rospy.Publisher('/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=1, latch=True)
            info_msg = CarlaEgoVehicleInfo()
            for wheel in data['wheels']:
                wheel_info = CarlaEgoVehicleInfoWheel()
                wheel_info.tire_friction = wheel['tire_friction']
                wheel_info.damping_rate = wheel['damping_rate']
                wheel_info.steer_angle = wheel['steer_angle']
                wheel_info.disable_steering = wheel['disable_steering']
                info_msg.wheels.append(wheel_info)
            info_msg.max_rpm = data['max_rpm']
            info_msg.moi = data['moi']
            info_msg.damping_rate_full_throttle = data['damping_rate_full_throttle']
            info_msg.damping_rate_zero_throttle_clutch_engaged
            info_msg.damping_rate_zero_throttle_clutch_disengaged = data['damping_rate_zero_throttle_clutch_disengaged']
            info_msg.use_gear_autobox = data['use_gear_autobox']
            info_msg.clutch_strength = data['clutch_strength']
            info_msg.mass = data['mass']
            info_msg.drag_coefficient = data['drag_coefficient']
            info_msg.center_of_mass.x = data['center_of_mass']['x']
            info_msg.center_of_mass.y = data['center_of_mass']['y']
            info_msg.center_of_mass.z = data['center_of_mass']['z']
            self.vehicle_info_publisher.publish(info_msg)
        msg = CarlaEgoVehicleStatus()
        msg.header = self.get_header()
        msg.velocity = data['speed']
        self.speed = data['speed']
        #todo msg.acceleration
        msg.control.throttle = self.current_control.throttle
        msg.control.steer = self.current_control.steer
        msg.control.brake = self.current_control.brake
        msg.control.hand_brake = self.current_control.hand_brake
        msg.control.reverse = self.current_control.reverse
        msg.control.gear = self.current_control.gear
        msg.control.manual_gear_shift = self.current_control.manual_gear_shift

        self.vehicle_status_publisher.publish(msg)

    def publish_hd_map(self, sensor_id, data):
        """
        publish hd map data
        """
        roll = -math.radians(data['transform']['roll'])
        pitch = -math.radians(data['transform']['pitch'])
        yaw = -math.radians(data['transform']['yaw'])
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        x = data['transform']['x']
        y = -data['transform']['y']
        z = data['transform']['z']

        if self.odometry_publisher:

            odometry = Odometry()
            odometry.header.frame_id = 'map'
            odometry.header.stamp = rospy.Time.from_sec(self.timestamp)
            odometry.child_frame_id = 'base_link'
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            odometry.pose.pose.position.z = z

            odometry.pose.pose.orientation.x = quat[0]
            odometry.pose.pose.orientation.y = quat[1]
            odometry.pose.pose.orientation.z = quat[2]
            odometry.pose.pose.orientation.w = quat[3]

            odometry.twist.twist.linear.x = self.speed
            odometry.twist.twist.linear.y = 0
            odometry.twist.twist.linear.z = 0

            self.odometry_publisher.publish(odometry)

        if self.world_info_publisher:
            #extract map name
            map_name = os.path.basename(data['map_file'])[:-4] 
            if self.current_map_name != map_name:
                self.current_map_name = map_name
                world_info = CarlaWorldInfo()
                world_info.map_name = self.current_map_name
                world_info.opendrive = data['opendrive']
                self.world_info_publisher.publish(world_info)
        if self.map_file_publisher:
            self.map_file_publisher.publish(data['map_file'])

    def calibrate_lonlat0(self, route_gps, route_world_coordinate):

        if route_gps is None or len(route_gps) != len(route_world_coordinate):
            return

        if len(route_world_coordinate) > 101:
            end_id = 100
        else:
            end_id = len(route_world_coordinate)-2
        
        if end_id < 1:
            return

        gps_1 = route_gps[0][0]
        gps_2 = route_gps[end_id][0]

        lon1 = gps_1.get('lon')
        lat1 = gps_1.get('lat')

        lon2 = gps_2.get('lon')
        lat2 = gps_2.get('lat')

        world_coordinate_1 = route_world_coordinate[0][0]
        world_coordinate_2 = route_world_coordinate[end_id][0]

        x1 = world_coordinate_1.location.x
        y1 = world_coordinate_1.location.y
        x2 = world_coordinate_2.location.x
        y2 = world_coordinate_2.location.y 

        EARTH_RADIUS_EQUA = 6378137.0

        tx1 = math.radians(lon1)*EARTH_RADIUS_EQUA
        ty1 = EARTH_RADIUS_EQUA* math.log(math.tan((90+lat1)*math.pi/360))

        tx2 = math.radians(lon2)*EARTH_RADIUS_EQUA
        ty2 = EARTH_RADIUS_EQUA* math.log(math.tan((90+lat2)*math.pi/360))

        tx0 = (x2*tx1-x1*tx2)/(x2-x1)
        ty0 = (y2*ty1-y1*ty2)/(y2-y1)

        self.lon_origin = tx0/EARTH_RADIUS_EQUA/math.pi*180
        self.lat_origin = math.atan(math.exp(ty0/EARTH_RADIUS_EQUA))/math.pi*360-90

    def gps_to_world(self, lon, lat):

        EARTH_RADIUS_EQUA = 6378137.0
        lon_0 = self.lon_origin
        lat_0 = self.lat_origin

        scale = math.cos(math.radians(lat_0))
        mx0 = scale*math.radians(lon_0)*EARTH_RADIUS_EQUA
        my0 = scale*EARTH_RADIUS_EQUA* math.log(math.tan((90+lat_0)*math.pi/360))

        mx = scale*math.radians(lon)*EARTH_RADIUS_EQUA
        my = scale*EARTH_RADIUS_EQUA* math.log(math.tan((90+lat)*math.pi/360))

        return mx-mx0, my-my0

    def get_object(self, target_vehicle_id, target_vehicle):

        # Get the vehicle location
        t_gps = target_vehicle.get('position')
        t_loc = self.gps_to_world(t_gps[1], t_gps[0])

        # Get vehicle direction
        rotation = target_vehicle.get('orientation')
        roll, pitch, yaw = rotation
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # Get vehicle speed from last time vehicle list
        matched_vehicle_state = None
        if target_vehicle_id in self.objects_storage:
            matched_vehicle_state = self.objects_storage[target_vehicle_id]

        if matched_vehicle_state is None:
            speed_x = speed_y = 0
            speed = 0
        else:
            speed_x = (t_loc[0] - matched_vehicle_state[0]) / 0.05
            speed_y = (t_loc[1] - matched_vehicle_state[1]) / 0.05
            speed = math.sqrt(speed_x*speed_x + speed_y*speed_y)
            if speed > 1 and speed > max(matched_vehicle_state[2] * 1.5, matched_vehicle_state[2] + 6 * 0.05):
                speed = speed / 2
            if speed < min(matched_vehicle_state[2] * 0.5, matched_vehicle_state[2] - 6 * 0.05):
                speed = matched_vehicle_state[2]

        # wrap vehicle info
        vehicle_target = TrackingBox()
        vehicle_target.uid = target_vehicle_id

        vehicle_target.bbox.pose.pose.position.x = t_loc[0]
        vehicle_target.bbox.pose.pose.position.y = -t_loc[1]
        vehicle_target.bbox.pose.pose.position.z = 0
        vehicle_target.bbox.pose.pose.orientation.x = quaternion[0]
        vehicle_target.bbox.pose.pose.orientation.y = quaternion[1]
        vehicle_target.bbox.pose.pose.orientation.z = quaternion[2]
        vehicle_target.bbox.pose.pose.orientation.w = quaternion[3]
        vehicle_target.bbox.dimension.length_x = 1
        vehicle_target.bbox.dimension.length_y = 1
        vehicle_target.bbox.dimension.length_z = 1

        vehicle_target.twist.twist.linear.x = speed_x
        vehicle_target.twist.twist.linear.y = -speed_y
        vehicle_target.twist.twist.linear.z = 0
        
        self.objects_storage[target_vehicle_id] = (t_loc[0], t_loc[1], speed)
        return vehicle_target

    def publish_objects(self, sensor_id, data):
        vehicle_list_layout         = data.get('vehicles')
        ego_vehicle_layout          = data.get('hero_vehicle')
        pedestrian_list_layout      = data.get('walkers')
        # speed_limit_layout        = data.get('speed_limits')
        # traffic_lights_layout     = data.get('traffic_lights')
        # stop_signs_layout         = data.get('stop_signs')
        # static_obstacles_layout   = data.get('static_obstacles')

        targets = TrackingBoxArray()
        targets.header = self.get_header()
        targets.header.frame_id = "map"

        ego_vehicle_id = ego_vehicle_layout.get('id')
        for target_vehicle_id in vehicle_list_layout:
            # if the vehicle is ego vehicle?
            if ego_vehicle_id == target_vehicle_id:
                continue

            target_vehicle = vehicle_list_layout[target_vehicle_id]
            vehicle_target = self.get_object(target_vehicle_id, target_vehicle)

            target_cls = ObjectClass()
            target_cls.classid = ObjectClass.VEHICLE
            target_cls.score = 1
            vehicle_target.classes.append(target_cls)
            targets.targets.append(vehicle_target)

        for target_ped_id in pedestrian_list_layout:
            target_ped = pedestrian_list_layout[target_ped_id]
            ped_target = self.get_object(target_ped_id, target_ped)

            target_cls = ObjectClass()
            target_cls.classid = ObjectClass.HUMAN_PEDESTRIAN
            target_cls.score = 1
            ped_target.classes.append(target_cls)
            targets.targets.append(ped_target)

        self.objects_publisher.publish(targets)

    def run_step(self, input_data, timestamp):
        self.vehicle_control_event.clear()
        self.timestamp = timestamp
        self.clock_publisher.publish(Clock(rospy.Time.from_sec(timestamp)))

        #check if stack is still running
        if self.stack_process and self.stack_process.poll() is not None:
            raise RuntimeError("Stack exited with: {} {}".format(self.stack_process.returncode, self.stack_process.communicate()[0]))

        #publish global plan to ROS once
        if self._global_plan_world_coord and not self.global_plan_published:
            self.global_plan_published = True
            self.publish_plan()

        if not self.global_plan_calibrated:
            if self._global_plan and self._global_plan_world_coord:
                self.calibrate_lonlat0(self._global_plan, self._global_plan_world_coord)
                self.global_plan_calibrated = True

        new_data_available = False

        #publish data of all sensors
        for key, val in input_data.items():
            new_data_available = True
            sensor_type = self.id_to_sensor_type_map[key]
            if sensor_type == 'sensor.camera.rgb':
                self.publish_camera(key, val[1])
            elif sensor_type == 'sensor.lidar.ray_cast':
                self.publish_lidar(key, val[1])
            elif sensor_type == 'sensor.other.gnss':
                self.publish_gnss(key, val[1])
            elif sensor_type == 'sensor.can_bus':
                self.publish_can(key, val[1])
            elif sensor_type == 'sensor.hd_map':
                self.publish_hd_map(key, val[1])
            elif sensor_type == 'sensor.object_finder':
                self.publish_objects(key, val[1])
            else:
                raise TypeError("Invalid sensor type: {}".format(sensor_type))

        if self.use_stepping_mode:
            if self.step_mode_possible and new_data_available:
                self.vehicle_control_event.wait()
        # if the stepping mode is not used or active, there is no need to wait here

        return self.current_control

    def sensors(self):
        sensors = [
            {'type':'sensor.camera.rgb', 'x': 0, 'y': 0, 'z': 2, 'roll': 0.0, 'pitch': 4, 'yaw': 0.0, #pitch 3 for traffic light
             'width': 400, 'height': 300, 'fov': 10, 'id': 'Telephoto'},
            {'type':'sensor.camera.rgb', 'x': 0, 'y': 0, 'z': 2, 'roll': 0.0, 'pitch': -1, 'yaw': 0.0, #pitch 3 for traffic light
             'width': 600, 'height': 400, 'fov': 60, 'id': 'Wideangle'},
            {'type': 'sensor.lidar.ray_cast', 'x': 0.0, 'y': 0.0, 'z': 2.5, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'id': 'Lidar'},
            {'type': 'sensor.other.gnss', 'x': 0.0, 'y': -0.0, 'z': 1.60, 'id': 'GPS'},
            {'type': 'sensor.other.gnss', 'x': 1.0, 'y': -0.0, 'z': 1.60, 'id': 'GPS_Helper'},
            {'type': 'sensor.can_bus', 'reading_frequency': 25, 'id': 'can_bus'},
            {'type': 'sensor.hd_map', 'reading_frequency': 25, 'id': 'hdmap'},
            {'type': 'sensor.object_finder', 'reading_frequency': 20, 'id': 'fobject'},
        ]
        return sensors
