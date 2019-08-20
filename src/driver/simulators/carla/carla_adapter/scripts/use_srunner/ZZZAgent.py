# This script is for running Carla scenario_runner

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.challenge.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.challenge.autoagents.ros_agent import RosAgent

import numpy as np

class ZZZAgent(RosAgent, object):
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
            {'type': 'sensor.hd_map', 'reading_frequency': 1, 'id': 'hdmap'},
        ]
        return sensors

    def use_stepping_mode(self):
        return True
