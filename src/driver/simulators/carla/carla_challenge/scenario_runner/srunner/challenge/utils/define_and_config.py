#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Common_Define: provide static var
Common_Config: provide config var
"""

class Common_Define(object):
    first_path = ''
    second_path = ''
    third_path = ''
    scenario_type = ''
    velocity_filename = ''
    vehicle_location_filename = ''
    ref_waypoint_filename = ''
    stop_update = False
    follow_speed = 2
    cruise_speed = 4
    first_vehicle_location = 15     # first_vehicle spawn location ahead ego-vehicle 
    remove_obstacle_time = 60       # in AheadObstacle scenario ,first_vehicle is removed after remove_obstacle_time(seconds)
    repetition_num = 35
    excel_rows = 1
    excel_value = []
    save_excel = True
    # excel_value = {'num' : '','route_name' : '', 'route_length' : 0.0, 'left_turn' : 0, 'right_turn' : '', 'collision_static' : 0,
    #  'collision_vehicle' : 0,'collision_pedestrian' : 0, 'red_light' : 0,'wrong_way' : 0,'route_deviation' : 0,'sidewalk_invasion' : 0,
    #  'stop_infraction' : 0,'scenario_type' : ''}
    left_turn_num , right_turn_num = 0, 0
    COLLISION_STATIC_NUM, COLLISION_VEHICLE_NUM, COLLISION_VEHICLE_NUM = 0, 0, 0
    TRAFFIC_LIGHT_NUM, WRONG_WAY_NUM, ROUTE_DEVIATION, SIDEWALK_INVASION_NUM, STOP_NUM = 0, 0 ,0 ,0 , 0

class Common_Config(object):
    routes_follow_speed = [2,2,2,2,2]    # first_vehicle follow speed for every_route
    routes_cruise_speed = [4,4,4,4,4]    # first_vehicle cruise speed for every_route
    routes_first_vehicle_location = [15, 15, 15, 15,15]     # first_vehicle spawn location ahead ego-vehicle for every_route
