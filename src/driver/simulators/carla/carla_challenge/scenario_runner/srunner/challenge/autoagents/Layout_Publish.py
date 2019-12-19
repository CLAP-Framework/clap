## Read and publish Layout


import os
use_ros = os.environ.has_key("IN_ROS")

import carla
import math
import matplotlib.path as mplPath
import numpy as np
from .SurroundingObjects import Surrounding_pedestrian, Surrounding_vehicle
from GPS_tool import *
from carlaok_tool import *


if use_ros:
    import rospy
    import threading
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float32
    from mobileye_560_660_msgs.msg import ObstacleData, ObstacleDataArray
    import rosgraph


global vehicle_list,sensor_range
vehicle_list = []
sensor_range = 30
dt = 0.05

"""
Vehicle

"""

def publish_vehicle_list():
    if not use_ros:
        return

    msg = ObstacleDataArray()

    global vehicle_list
    for vehicle in vehicle_list:
        vehicle_msg = ObstacleData()
        vehicle_msg.obstacle_id = vehicle.id
        vehicle_msg.obstacle_pos_x = vehicle.location.x
        vehicle_msg.obstacle_pos_y = vehicle.location.y
        vehicle_msg.obstacle_type = 0
        vehicle_msg.obstacle_yaw = vehicle.yaw
        vehicle_msg.obstacle_speed = vehicle.speed
        msg.obstacles.append(vehicle_msg)

    vehicle_list_publisher = rospy.Publisher(
            '/carla/environment_perception/vehicle_list', ObstacleDataArray, queue_size=1, latch=True)
    vehicle_list_publisher.publish(msg)


def get_vehicle_list():

    global vehicle_list
    return vehicle_list


def update_surrounding_vehicle_list(input_data):

    surrounding_vehicle_list = []
    global sensor_range

    for key, val in input_data.items():            
        if key == 'object_finder':
            vehicle_list_layout = val[1].get('vehicles')
            ego_vehicle_layout = val[1].get('hero_vehicle')

            # pedestrian_list_layout = val[1].get('walkers')
            # speed_limit_layout = val[1].get('speed_limits')
            # traffic_lights_layout = val[1].get('traffic_lights')
            # stop_signs_layout = val[1].get('stop_signs')
            # static_obstacles_layout = val[1].get('static_obstacles')

    ego_vehicle_id = ego_vehicle_layout.get('id')
    ego_gps = ego_vehicle_layout.get('position')
    ego_vehicle_location = GpsToWorldCoordinate(ego_gps[1],ego_gps[0])

    for target_vehicle_id in vehicle_list_layout:

        # if the vehicle is ego vehicle?
        target_vehicle = vehicle_list_layout[target_vehicle_id]
        if ego_vehicle_id == target_vehicle_id:
            continue

        # Get the vehicle location
        t_gps = target_vehicle.get('position')
        t_loc = GpsToWorldCoordinate(t_gps[1],t_gps[0])

        # if this vehicle is far away?
        d = distance_between_two_loc(t_loc,ego_vehicle_location)
        if d > sensor_range:
            continue

        # Get vehicle direction
        rotation = target_vehicle.get('orientation')
        yaw = rotation[2]
        v_vec = ([math.cos(math.radians(yaw)),math.sin(math.radians(yaw)),0.0])

        # Get vehicle speed from last time vehicle list
        matched_vehicle = None
        for vehicle in vehicle_list:
            if vehicle.id == target_vehicle_id:
                matched_vehicle = vehicle
                break

        if matched_vehicle is None:
            speed = 0
            add_vehicle = Surrounding_vehicle()
        else:
            add_vehicle = matched_vehicle
            last_loc = matched_vehicle.location
            d_in_dt = distance_between_two_loc(t_loc,last_loc)
            speed = d_in_dt/dt
            if speed > 1 and speed > max(matched_vehicle.speed*1.5,matched_vehicle.speed + 6*dt):
                speed = speed/2
            if speed < min(matched_vehicle.speed*0.5,matched_vehicle.speed - 6*dt):
                speed = matched_vehicle.speed

        # wrap vehicle info
        add_vehicle.location = t_loc
        add_vehicle.speed = speed
        add_vehicle.speed_direction = v_vec
        add_vehicle.yaw = yaw
        add_vehicle.id = target_vehicle_id
        add_vehicle.trajectory_buffer.append(t_loc)
        
        surrounding_vehicle_list.append(add_vehicle)

    global vehicle_list
    vehicle_list = surrounding_vehicle_list

"""
pedestrian

"""
global pedestrian_list
pedestrian_list = []

def publish_pedestrian_list():
    if not use_ros:
        return

    msg = ObstacleDataArray()

    global pedestrian_list
    for pedestrian in pedestrian_list:
        pedestrian_msg = ObstacleData()
        pedestrian_msg.obstacle_id = pedestrian.id
        pedestrian_msg.obstacle_pos_x = pedestrian.location.x
        pedestrian_msg.obstacle_pos_y = pedestrian.location.y
        pedestrian_msg.obstacle_type = 3
        pedestrian_msg.obstacle_yaw = pedestrian.yaw
        pedestrian_msg.obstacle_speed = pedestrian.speed
        msg.obstacles.append(pedestrian_msg)

    pedestrian_list_publisher = rospy.Publisher(
            '/carla/environment_perception/pedestrian_list', ObstacleDataArray, queue_size=1, latch=True)
    pedestrian_list_publisher.publish(msg)

def get_pedestrian_list():

    global pedestrian_list
    return pedestrian_list

def update_pedestrian_list(input_data):

    global sensor_range
    surrounding_pedestrian_list = []

    for key, val in input_data.items():            
        if key == 'object_finder':
            pedestrian_list_layout = val[1].get('walkers')
            ego_vehicle_layout = val[1].get('hero_vehicle')

            # vehicle_list_layout = val[1].get('vehicles')
            # ego_vehicle_layout = val[1].get('hero_vehicle')
            # pedestrian_list_layout = val[1].get('walkers')
            # speed_limit_layout = val[1].get('speed_limits')
            # traffic_lights_layout = val[1].get('traffic_lights')
            # stop_signs_layout = val[1].get('stop_signs')
            # static_obstacles_layout = val[1].get('static_obstacles')

    ego_gps = ego_vehicle_layout.get('position')
    ego_vehicle_location = GpsToWorldCoordinate(ego_gps[1],ego_gps[0])

    for target_pedestrian_id in pedestrian_list_layout:
        target_pedestrian = pedestrian_list_layout[target_pedestrian_id]

        # Get the pedestrian location
        t_gps = target_pedestrian.get('position')
        t_loc = GpsToWorldCoordinate(t_gps[1],t_gps[0])

        # if this pedestrian is far away?
        d = distance_between_two_loc(t_loc,ego_vehicle_location)
        if d > sensor_range:
            continue

        # Get pedestrian direction
        rotation = target_pedestrian.get('orientation')
        yaw = rotation[2]
        v_vec = ([math.cos(math.radians(yaw)),math.sin(math.radians(yaw)),0.0])

        # Get pedestrian speed from last time pedestrian list
        matched_pedestrian = None
        global pedestrian_list

        for pedestrian in pedestrian_list:
            if pedestrian.id == target_pedestrian_id:
                matched_pedestrian = pedestrian
                break

        if matched_pedestrian is None:
            speed = 0
            add_pedestrian = Surrounding_pedestrian()
        else:
            add_pedestrian = matched_pedestrian
            last_loc = matched_pedestrian.location
            d_in_dt = distance_between_two_loc(t_loc,last_loc)
            speed = d_in_dt/dt
            if speed > 1 and speed > max(matched_pedestrian.speed*1.5,matched_pedestrian.speed + 6*dt):
                speed = speed/2
            if speed < min(matched_pedestrian.speed*0.5,matched_pedestrian.speed - 6*dt):
                speed = matched_pedestrian.speed

        # wrap pedestrian info
        add_pedestrian.location = t_loc
        add_pedestrian.speed = speed
        add_pedestrian.speed_direction = v_vec
        add_pedestrian.yaw = yaw
        add_pedestrian.id = target_pedestrian_id
        
        surrounding_pedestrian_list.append(add_pedestrian)

    pedestrian_list = surrounding_pedestrian_list



"""
Speed Limit
"""

global speed_limit

speed_limit = 30

def publish_speed_limit():
    if not use_ros:
        return

    msg = Float32()

    global speed_limit
    
    msg.data = speed_limit

    speed_limit_publisher = rospy.Publisher(
            '/carla/environment_perception/speed_limit', Float32, queue_size=1, latch=True)
    speed_limit_publisher.publish(msg)


def get_speed_limit():

    global speed_limit
    return speed_limit

def update_speed_limit(input_data):
    for key, val in input_data.items():            
        if key == 'object_finder':
            speed_limit_layout = val[1].get('speed_limits')
            ego_vehicle_layout = val[1].get('hero_vehicle')
            # pedestrian_list_layout = val[1].get('walkers')
            # ego_vehicle_layout = val[1].get('hero_vehicle')
            # vehicle_list_layout = val[1].get('vehicles')
            # ego_vehicle_layout = val[1].get('hero_vehicle')
            # pedestrian_list_layout = val[1].get('walkers')
            # speed_limit_layout = val[1].get('speed_limits')
            # traffic_lights_layout = val[1].get('traffic_lights')
            # stop_signs_layout = val[1].get('stop_signs')
            # static_obstacles_layout = val[1].get('static_obstacles')
        
    ego_gps = ego_vehicle_layout.get('position')
    ego_vehicle_location = GpsToWorldCoordinate(ego_gps[1],ego_gps[0])

    nearest_speed_limit = -1
    global sensor_range
    nearest_distance = sensor_range

    for target_speed_limit_id in speed_limit_layout:

        target_speed_limit = speed_limit_layout[target_speed_limit_id]

        # Get the speed_limit location
        t_gps = target_speed_limit.get('position')
        t_loc = GpsToWorldCoordinate(t_gps[1],t_gps[0])

        # if this speed_limit is far away?
        d = distance_between_two_loc(t_loc,ego_vehicle_location)
        if d < nearest_distance:
            nearest_distance = d
            nearest_speed_limit = target_speed_limit.get('speed')

        global speed_limit
        if nearest_speed_limit > 0:
            speed_limit = nearest_speed_limit
        else:
            speed = 30
    

"""
Traffic Lights
"""

def update_traffic_light(input_data,local_path):
    for key, val in input_data.items():            
        if key == 'object_finder':
            traffic_lights_layout = val[1].get('traffic_lights')
            ego_vehicle_layout = val[1].get('hero_vehicle')
        
    ego_gps = ego_vehicle_layout.get('position')
    ego_vehicle_location = GpsToWorldCoordinate(ego_gps[1],ego_gps[0])

    nearest_distance = -1
    nearest_id = None
    nearest_state = None
    nearest_trigger_valume = None

    for target_traffic_light_id in traffic_lights_layout:

        target_traffic_light = traffic_lights_layout[target_traffic_light_id]

        # Get the traffic_light location
        t_gps = target_traffic_light.get('position')
        t_loc = GpsToWorldCoordinate(t_gps[1],t_gps[0])
        t_state = target_traffic_light.get('state')
        trigger_volume_gps = target_traffic_light.get('trigger_volume')

        d = distance_between_two_loc(t_loc,ego_vehicle_location)
        if d > 200:
            continue

        trigger_valume = []
        for gps in trigger_volume_gps:
            corner_loc = GpsToWorldCoordinate(gps[0],gps[1])
            trigger_valume.append(corner_loc)
        
        bbPath = mplPath.Path(np.array([[trigger_valume[0].x, trigger_valume[0].y],
                     [trigger_valume[1].x, trigger_valume[1].y],
                     [trigger_valume[2].x, trigger_valume[2].y],
                     [trigger_valume[3].x, trigger_valume[3].y],
                     [trigger_valume[4].x, trigger_valume[4].y]]))
        
        # if the local_path drive in a traffic light area
        for (waypoint,_) in local_path:
            if bbPath.contains_point((waypoint.transform.location.x, waypoint.transform.location.y)):
                # if is the neareset traffic light
                distance_before_traffic_light = distance_between_two_loc(ego_vehicle_location,waypoint.transform.location)
                if nearest_distance < distance_before_traffic_light:
                    nearest_distance = distance_before_traffic_light
                    nearest_state = t_state
                    nearest_id = target_traffic_light_id
                    nearest_trigger_valume = trigger_valume
                break

    return nearest_trigger_valume,nearest_id,nearest_state,nearest_distance
            
""" 
Stop Sign
"""


def update_stop_sign(input_data,local_path):
    for key, val in input_data.items():            
        if key == 'object_finder':
            stop_signs_layout = val[1].get('stop_signs')
            ego_vehicle_layout = val[1].get('hero_vehicle')
        
    ego_gps = ego_vehicle_layout.get('position')
    ego_vehicle_location = GpsToWorldCoordinate(ego_gps[1],ego_gps[0])

    nearest_distance = -1
    nearest_id = None
    nearest_state = None
    nearest_trigger_valume = None

    for target_stop_sign_id in stop_signs_layout:

        target_stop_sign = stop_signs_layout[target_stop_sign_id]

        # Get the stop_sign location
        t_gps = target_stop_sign.get('position')
        t_loc = GpsToWorldCoordinate(t_gps[1],t_gps[0])
        trigger_volume_gps = target_stop_sign.get('trigger_volume')

        d = distance_between_two_loc(t_loc,ego_vehicle_location)
        if d > 200:
            continue

        trigger_valume = []
        for gps in trigger_volume_gps:
            corner_loc = GpsToWorldCoordinate(gps[0],gps[1])
            trigger_valume.append(corner_loc)
        
        bbPath = mplPath.Path(np.array([[trigger_valume[0].x, trigger_valume[0].y],
                     [trigger_valume[1].x, trigger_valume[1].y],
                     [trigger_valume[2].x, trigger_valume[2].y],
                     [trigger_valume[3].x, trigger_valume[3].y],
                     [trigger_valume[4].x, trigger_valume[4].y]]))
        
        # if the local_path drive in a traffic light area
        for (waypoint,_) in local_path:
            if bbPath.contains_point((waypoint.transform.location.x, waypoint.transform.location.y)):
                # if is the neareset traffic light
                distance_before_stop_sign = distance_between_two_loc(ego_vehicle_location,waypoint.transform.location)
                if nearest_distance < distance_before_stop_sign:
                    nearest_distance = distance_before_stop_sign
                    nearest_id = target_stop_sign_id
                    nearest_trigger_valume = trigger_valume
                break

    return nearest_trigger_valume,nearest_id,nearest_distance