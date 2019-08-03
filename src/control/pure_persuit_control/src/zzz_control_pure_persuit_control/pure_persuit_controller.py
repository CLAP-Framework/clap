#!/usr/bin/env python

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math
from zzz_control_msgs.msg import ControlCommand
import numpy as np
import tf
from zzz_common.geometry import *
import rospy

class PurePersuitController():
    """
    PurePersuitController is the combination of a PID controller for longitudinal control and a pure persuilt controller for lateral control
    to perform the low level control a vehicle from client side
    """

    def __init__(self):
        """
        :param vehicle: actor to apply to local planner logic onto
        """
        self._lon_controller = PIDLongitudinalController()
        self._lat_controller = PurePersuitLateralController()
        self.ego_vehicle_pose = None
        self.ego_vehicle_speed = 0
        self.desired_trajectory = None
        self.desired_speed = 30.0

    def update_decision(self,decision):
        self.desired_trajectory = decision.trajectory
        self.desired_speed = decision.desired_speed

    def update_ego_vehicle_speed(self,speed):
        self.ego_vehicle_speed = speed

    def ready_for_control(self, short_distance_thres = 5):
        if self.desired_trajectory is None or len(self.desired_trajectory.poses) == 0:
            rospy.logdebug("Haven't recevied trajectory")
            return False

        last_loc = np.array([self.desired_trajectory.poses[-1].pose.position.x,self.desired_trajectory.poses[-1].pose.position.y]) 
        ego_loc = np.array([self.ego_state.pose.pose.position.x,self.ego_state.pose.pose.position.y])
        d = np.linalg.norm(ego_loc-last_loc)

        if d < short_distance_thres:
            return False
        
        return True

    def run_step(self, pose):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to track a trajectory
        at a given target_speed.
        return: control
        """
        rospy.logdebug("received target speed:%f, current_speed: %f",self.desired_speed,self.ego_vehicle_speed)

        self.ego_vehicle_pose = pose

        if not self.ready_for_control():
            control_msg = ControlCommand()
            control_msg.throttle = 0
            control_msg.brake = 1
            control_msg.steer = 0
            
            return control_msg
        
        target_speed = self.desired_speed
        trajectory = self.desired_trajectory
        current_speed = self.ego_vehicle_speed
        ego_pose = self.ego_vehicle_pose

        throttle, brake = self._lon_controller.run_step(target_speed, current_speed)
        steer = self._lat_controller.run_step(ego_pose, trajectory, current_speed)

        rospy.logdebug("throttle = %f, brake = %f, steer = %f", throttle, brake, steer)

        control_msg = ControlCommand()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steer = steer

        return control_msg


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
        self._dt = 0.05 # TODO: timestep
        self._integ = 0.0
        self._e_buffer = deque(maxlen=30)

    def run_step(self, target_speed, current_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
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


class PurePersuitLateralController():
    """
    PurePersuitLateralController implements lateral control using a PurePersuit Control.
    """

    def __init__(self):
        """

        """
        pass

    def run_step(self, ego_pose, trajectory, current_speed):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """

        control_point = self._control_point(ego_pose,trajectory,current_speed)

        return self._purepersuit_control(control_point, ego_pose)

    def _control_point(self,ego_pose,trajectory,current_speed,resolution = 0.1):

        if current_speed > 10:        
            control_target_dt = 0.5 - (current_speed-10)*0.01
        else:
            control_target_dt = 0.5
     
        control_target_distance = control_target_dt * current_speed  ## m
        if control_target_distance < 3:
            control_target_distance = 3

        ego_loc = np.array([ego_pose.position.x,ego_pose.position.y])

        trajectory_array = self.convert_trajectory_to_ndarray(trajectory)

        # rospy.logdebug("control trajectory: %s",str(trajectory_array))
        trajectory_dense = dense_polyline(trajectory_array,resolution)

        end_idx = self.get_next_idx(ego_loc, trajectory_dense, control_target_distance)
        wp_loc = trajectory_dense[end_idx]

        return wp_loc


    def _purepersuit_control(self, waypoint, ego_pose):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """

        orientation = ego_pose.orientation
        _, _, yaw_r = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        ego_yaw = -math.degrees(yaw_r)

        ego_x = ego_pose.position.x
        ego_y = ego_pose.position.y

        # v_begin = vehicle_transform.location
        # v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
        #                                  y=math.sin(math.radians(vehicle_transform.rotation.yaw)))
        # v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        
        v_vec = np.array([math.cos(math.radians(ego_yaw)),
                          math.sin(math.radians(ego_yaw)),
                          0.0])

        target_x = waypoint[0]
        target_y = waypoint[1]

        w_vec = np.array([target_x-
                          ego_x, target_y -
                          ego_y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                         (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        lf = 1.2
        lr = 1.65
        lwb = lf+lr
        
        v_rear_x = ego_x - v_vec[0]*lr/np.linalg.norm(v_vec)
        v_rear_y = ego_y - v_vec[1]*lr/np.linalg.norm(v_vec)
        l = (target_x-v_rear_x)*(target_x-v_rear_x)+(target_y-v_rear_y)*(target_y-v_rear_y)
        l = math.sqrt(l)

        theta = np.arctan(2*np.sin(_dot)*lwb/l)
        

        k = 1# np.pi/180*50
        theta = theta*k
        return theta


    # TODO: Move into library

    def convert_trajectory_to_ndarray(self,trajectory):
        trajectory_array = [(pose.pose.position.x, pose.pose.position.y) for pose in trajectory.poses]
        return np.array(trajectory_array)

    def get_idx(self,loc,trajectory):

        
        dist = np.linalg.norm(trajectory-loc,axis=1)
        idx = np.argmin(dist)
        return idx

    def get_next_idx(self,start_loc,trajectory,distance):

        start_idx = self.get_idx(start_loc,trajectory)
        dist_list = np.cumsum(np.linalg.norm(np.diff(trajectory,axis = 0),axis = 1))
        for end_idx in range(start_idx,len(trajectory)-1):
            if dist_list[end_idx] > dist_list[start_idx] + distance:
                return end_idx

        rospy.logdebug("start_idx: %s, ego_loc: %s, trajectory: %s", 
                                                    str(start_idx),
                                                    str(start_loc),
                                                    str(trajectory))
                                                    
    
