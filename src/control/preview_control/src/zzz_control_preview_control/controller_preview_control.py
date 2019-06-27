#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math

import numpy as np

import carla
from agents.tools.misc import get_speed
from .EnvironmentState import EnvironmentState


class PreviewController():
    """
    PreviewController is the combination of a PID controller for longitudinal control and a preview controller for lateral control
    to perform the low level control a vehicle from client side
    """

    def __init__(self):
        """
        :param vehicle: actor to apply to local planner logic onto
        """
        self._lon_controller = PIDLongitudinalController()
        self._lat_controller = PurePersuitLateralController()

    def run_step(self, target_speed, waypoint, EnvironmentInfo):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param current_speed: ego_vehicle speed
        :param dt: time differential in seconds
        :param ego_vehicle_transform

        :param waypoint: target location encoded as a waypoint
        :return: control
        """
        throttle, brake = self._lon_controller.run_step(target_speed, EnvironmentInfo)
        steering = self._lat_controller.run_step(waypoint, EnvironmentInfo)

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._K_P = 0.25/3.6
        self._K_D = 0.01
        self._K_I = 0.012
        self._dt = 0.05
        self._integ = 0.0
        self._e_buffer = deque(maxlen=30)

    def run_step(self, target_speed, EnvironmentInfo, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """
        self._dt = EnvironmentInfo.dt
        current_speed = EnvironmentInfo.ego_vehicle_speed  #m/s

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed, EnvironmentInfo)

    def _pid_control(self, target_speed, current_speed, EnvironmentInfo):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """

        target_speed = target_speed*3.6
        current_speed = current_speed*3.6

        _e = (target_speed - current_speed)
        self._integ += _e * self._dt
        self._e_buffer.append(_e)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = self._integ
            
        else:
            _de = 0.0
            _ie = 0.0
        
        kp = self._K_P
        ki = self._K_I
        kd = self._K_D

        if target_speed < 5:
            ki = 0
            kd = 0

        calculate_value = np.clip((kp * _e) + (kd * _de) + (ki * _ie), -1.0, 1.0)
        if calculate_value > 0:
            thr = calculate_value
            br = 0
        else:
            thr = 0
            br = -calculate_value

        return thr,br


class PreviewLateralController():
    """
    PreviewLateralController implements lateral control using a PurePersuit Control.
    """

    def __init__(self):
        """

        """
        pass

    def setup(self):
        """
        setting parameters
        """
        self.m = 1800 #kg
        self.Iz = 3270 #kg*m^2
        self.ks = 16
        self.lf = 1.2 #
        self.caf = 70000 #N/rad
        self.car = 60000 #N/rad
        self.dt = 0.05


    def run_step(self, waypoint, EnvironmentInfo):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._preview_control(waypoint, EnvironmentInfo)

    def _preview_control(self, waypoint, EnvironmentInfo):
        """
        Estimate the steering angle of the vehicle based on the preview control

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """

        vehicle_transform = EnvironmentInfo.ego_vehicle_transform
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])

        target_x = waypoint.x
        target_y = waypoint.y

        w_vec = np.array([target_x-
                          v_begin.x, target_y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                         (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        lf = 1.2
        lr = 1.65
        lwb = lf+lr
        
        v_rear_x = v_begin.x - v_vec[0]*lr/np.linalg.norm(v_vec)
        v_rear_y = v_begin.y - v_vec[1]*lr/np.linalg.norm(v_vec)
        l = (target_x-v_rear_x)*(target_x-v_rear_x)+(target_y-v_rear_y)*(target_y-v_rear_y)
        l = math.sqrt(l)

        theta = np.arctan(2*np.sin(_dot)*lwb/l)
        

        k = 1# np.pi/180*50
        theta = theta*k
        return theta

