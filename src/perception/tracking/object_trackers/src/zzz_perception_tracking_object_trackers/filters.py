'''
This module contains filters to estimate object states.

The property of pose state should be in format
    [x, y, z, vx, vy, vz, ax, ay, az,
    [0  1  2  3   4   5   6   7   8
     rx, ry, rz, wx, wy, wz, bx, by, bz]
     9   10  11  12  13  14  15  16  17 ]
where "b" denotes angular acceleration
'''
import numpy as np
import math
import filterpy.kalman as kf

import sys
import rospy

import zzz_common.dynamic_models as dm
from zzz_perception_msgs.msg import DetectionBoxArray, DetectionBox, ObjectClass
from zzz_perception_msgs.msg import TrackingBoxArray, TrackingBox
from geometry_msgs.msg import Pose

import tf

class Box_KF:
    '''
    Kalman Filter for a box shape estimation.
    In this case, the filter is actually a least squere estimator
    '''
    def __init__(self, Q=None):
        self._filter = kf.KalmanFilter(dim_x=3, dim_z=3)
        self._filter.F = self._filter.H = np.eye(3)

        if Q is None:
            self._filter.Q = np.diag([.01, .01, .01])
        else:
            assert Q.shape == (3,3)
            self._filter.Q = Q

        self._inited = False

    def predict(self, dt):
        if not self._inited:
            return
        self._filter.predict()

    def update(self, box):
        '''
        Box should have type of zzz_perception_msgs/DetectionBox.
        None input indicates that the tracker is failed to be associated.
        '''
        if box is not None:
            assert type(box) == DetectionBox
            
            z = np.array([
                box.bbox.dimension.length_x,
                box.bbox.dimension.length_y,
                box.bbox.dimension.length_z
            ])
            R = np.array(box.bbox.dimension.covariance).reshape(3,3)
            if not self._inited:
                self._filter.x = z
                self._filter.P = R
                
                self._inited = True
            else:
                self._filter.update(z, R=R)

    def state(self):
        raise NotImplementedError()

    def covariance(self):
        raise NotImplementedError()

class Pose_UKF_CTRA:
    '''
    UKF using CTRA model for pose estimation

    States: [x, y, rz, v, a, w]
            [0  1  2   3  4  5]
    Observe: [x, y, rz]
    '''
    def __init__(self, Q=None):
        self._filter = kf.UnscentedKalmanFilter(dim_x=6, dim_z=3,
            dt=None, fx=dm.motion_CTRA, hx=lambda s:s[:3],
            points=kf.MerweScaledSigmaPoints(6, alpha=.1, beta=.2, kappa=-1),
        )

        if Q is None:
            self._filter.Q = np.diag([1, 1, 1, 3, 10, 3])
        else:
            assert Q.shape == (6,6)
            self._filter.Q = Q

        self._inited = False 
        self._count_track = 0
        self._count_lost = 0

    def predict(self, dt):
        if not self._inited:
            return
        self._filter.predict(dt=dt)

    def update(self, box):
        '''
        Box should have type of zzz_perception_msgs/DetectionBox.
        None input indicates that the tracker is failed to be associated.
        '''
        if box is not None:
            assert type(box) == DetectionBox
            
            ori = box.bbox.pose.orientation
            _,_,rz = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            R = np.array(box.bbox.pose.covariance).reshape(6, 6)
            rz = dm.wrap_angle(rz)
            if not self._inited:
                self._filter.x = np.array([
                    box.bbox.pose.pose.x,
                    box.bbox.pose.pose.y,
                    rz,
                    0,
                    0,
                    0,
                ])
                self._filter.P = self._filter.Q
                self._filter.P[:3,:3] = R[(0,1,5), (0,1,5)] # select x, y, rz
                
                self._inited = True
            else:
                self._filter.update(np.array([
                    box.bbox.pose.pose.x,
                    box.bbox.pose.pose.y,
                    rz
                ]), R=R[(0,1,5), (0,1,5)])
                self._filter.x[2] = dm.wrap_angle(self._filter.x[2])

            self._count_track += 1
            self._count_lost = 0
        else:
            self._count_track = 0
            self._count_lost += 1
            
    @property
    def state(self):
        '''
        Return states in standard form
        '''
        s = np.zeros(18)
        s[:2] = self._filter.x[:2] # x, y
        s[11] = self._filter.x[2] # rz
        s[3] = self._filter.x[3] * np.cos(self._filter.x[2]) # vx
        s[4] = self._filter.x[3] * np.sin(self._filter.x[2]) # vy
        s[6] = self._filter.x[4] * np.cos(self._filter.x[2]) # ax
        s[7] = self._filter.x[4] * np.sin(self._filter.x[2]) # ay
        s[14] = self._filter.x[5]
        raise s

    @property
    def covariance(self):
        '''
        Return covariance of states in standard form

        # TODO: Return complete corresponding covariance. Hard to compute, use sampling method or directly sigma points?
        '''
        raise NotImplementedError()

class Pose_IMM:
    '''
    UKF using IMM (BR + CV + CA + CTRV + CTRA) model for pose estimation
    '''
    def __init__(self):
        raise NotImplementedError()

    def predict(self, dt):
        raise NotImplementedError()

    def update(self, z):
        raise NotImplementedError()

    def state(self):
        raise NotImplementedError()

    def covariance(self):
        raise NotImplementedError()

# class SimpleObj:
#     def __init__(self, xy_global=None, xy_local=None, classname=None, classid=None):
#         if xy_global is not None and xy_local is not None and classname is not None and classid is not None:
#             self.location = np.array( [xy_global[0,0], xy_global[1,0], 0] )
#             self.speed = 0
#             self.speed_direction = np.array([xy_local[0], xy_local[1], 0])
#             self.classname = classname
#             self.classid = classid
#         else:
#             self.location = None
#             self.speed = None
#             self.speed_direction = None
#             self.classname = None
#             self.classid = None

# class PercpVeh():
#     def __init__(self, id=None, x=None, y=None, vx=None, vy=None, simple_veh=None):

        # self.last_location = None #carla.Location(x=0, y=0, z=0)

        # self.matched = True
        # self.lost_count = 0

        # self.mot_noise = np.diag([1, 1, 3, 3])
        # self.obs_noise = np.diag([0.5, 0.5])
        # self.sigma = np.diag([1,1,50,50])

        # assert id is not None, "id should be given"
        # self.id = id
        # # self.time = time

        # if simple_veh is not None:
        #     self.state = np.array([simple_veh.location[0], simple_veh.location[1], 5, 5])[:,np.newaxis]
        #     self.classname = simple_veh.classname
        #     self.classid = simple_veh.classid
        # elif x is None:
        #     self.state = None
        # else:
        #     assert x is not None and y is not None, "Info not complete"
        #     if vx is None and vy is None:
        #         self.state = np.array([x, y, 0, 0])[:,np.newaxis]
        #     else:
        #         assert vx is not None and vy is not None, "velocity is not complete"
        #         self.state = np.array([x, y, vx, vy])[:,np.newaxis]

    # def update(self, simple_veh=None):
    #     if simple_veh is not None:
    #         x = simple_veh.location[0]
    #         y = simple_veh.location[1]
    #         self.filter(x, y)
    #         self.matched = True
    #         self.lost_count = 0
    #     else:
    #         self.matched = False
    #         self.lost_count += 1
    
    # def filter(self, x, y):
        
    #     self.last_location = self.location
    #     # dt = time - self.time
    #     dt = 0.05
    #     # print(dt)
    #     # self.time = time
    #     A = np.array([
    #         [1, 0, dt, 0],
    #         [0, 1, 0, dt],
    #         [0, 0, 1, 0],
    #         [0, 0, 0, 1]
    #     ])
    #     C = np.array([
    #         [1, 0, 0, 0],
    #         [0, 1, 0, 0]
    #     ])
    #     self.state = A.dot(self.state)
    #     self.sigma = A.dot(self.sigma).dot(A.T) + self.mot_noise

    #     z = np.array([[x], [y]]) - C.dot(self.state)
    #     S = self.obs_noise + C.dot(self.sigma).dot(C.T)
    #     K = self.sigma.dot(C.T).dot(np.linalg.inv(S))
    #     self.state += K.dot(z)
    #     self.sigma = (np.diag([1]*4) - K.dot(C)).dot(self.sigma)

    # @property
    # def location(self):
    #     return np.array([float(self.state[0,0]), float(self.state[1,0]), 1] )

    # @property
    # def speed(self):
    #     return 3.6 * math.sqrt(self.state[2,0]**2 + self.state[3,0]**2)

    # @property
    # def velo_x(self):
    #     return self.state[2, 0]

    # @property
    # def velo_y(self):
    #     return self.state[3, 0]

    # @property
    # def speed_direction(self):
    #     p_vec = np.array([self.state[2,0], self.state[3,0], 0.0])
    #     p_vec = p_vec/np.linalg.norm(p_vec)
    #     return p_vec


        
# def evalDist(veh1, veh2):
#     if veh2.matched == False:
#         xd = veh2.location[0] - veh1.location[0]
#         yd = veh2.location[1] - veh1.location[1]
#         dist = np.sqrt(xd**2 + yd**2)
#     else:
#         dist = None
#     return dist

class ObjectList(list):
    def __init__(self, lost_thresh = 3):
        self.last_id = 0
        self.lost_max = lost_thresh
    def matchto(self, veh_cur):
        min_dist = None
        i_best = None
        for i, veh in enumerate(self):
            dist = evalDist(veh_cur, veh)
            if dist is not None:
                if min_dist is None:
                    min_dist = dist
                    i_best = i
                elif dist < min_dist:
                    min_dist = dist
                    i_best = i
        if min_dist is not None:
            if min_dist < 10:
                self[i_best].update(veh_cur)
                return True
        return False

    def appendto(self, veh_cur):
        veh = PercpVeh(id=self.last_id, simple_veh=veh_cur)
        self.last_id += 1
        self.append(veh)

    def renewleft(self):
        if len(self) > 0:
            to_del_idx = []
            for i, veh in enumerate(self):
                if veh.matched == False:
                    veh.update()
                    if veh.lost_count >= self.lost_max:
                        to_del_idx.append(i)
            
            to_del_idx_sorted = sorted(to_del_idx, reverse = True)
            for i in to_del_idx_sorted:
                del self[i]
        
                    
    
    def updatelist(self, veh_list_cur):
        if len(self)>0:
            for veh in self:
                veh.matched = False
        if len(veh_list_cur) > 0:
            if len(self) > 0:
                for veh_cur in veh_list_cur:
                    found = self.matchto(veh_cur)
                    if not found:
                        self.appendto(veh_cur)
            else:
                for veh_cur in veh_list_cur:
                    self.appendto(veh_cur)
        self.renewleft()
        

class ObjectTracker:
    def __init__(self):
        self.initTrackList()
        self.initSelfPose()
        self.initRos()

    def initRos(self):
        rospy.init_node('object_tracking') # , anonymous=True
        rospy.Subscriber("/carla/object_detection", DetectionBox2DArray, self.callbackDetections)
        rospy.Subscriber("/carla/environment_perception/ego_vehicle_pose", Pose, self.callbackEgoPose)
        self.pub = rospy.Publisher("/carla/object_tracking", TrackingBoxArray, queue_size=10)
        rospy.spin()

    def initSelfPose(self):
        self.x = None
        self.y = None
        self.yaw = None

    def initTrackList(self):
        self.vehicle_list_cur = []
        self.pedestrian_list_cur = []

        self.vehicle_list = ObjectList()
        self.pedestrian_list = ObjectList()

    def callbackDetections(self, msg):
        self.gen_current_frame_list(msg.detections)

        self.vehicle_list.updatelist(self.vehicle_list_cur)
        self.pedestrian_list.updatelist(self.pedestrian_list_cur)

        self.rosSend()

    def callbackEgoPose(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        orientation = msg.orientation
        _, _, yaw_r = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw = -math.degrees(yaw_r)

    def rosSend(self):
        msg_sent = TrackingBoxArray()
        if len(self.vehicle_list) > 0:
            for obj in self.vehicle_list:
                msg_single = self.process_single_target_to_msg(obj)
                msg_sent.targets.append(msg_single)
        if len(self.pedestrian_list) > 0:
            for obj in self.pedestrian_list:
                msg_single = self.process_single_target_to_msg(obj)
                msg_sent.targets.append(msg_single)

        self.pub.publish(msg_sent)

    def process_single_target_to_msg(self, obj):
        msg_single = TrackingBox()
        msg_single.pose.pose.pose.position.x = obj.location[0]
        msg_single.pose.pose.pose.position.y = obj.location[1]
        msg_single.pose.pose.pose.position.z = obj.location[2]
        # msg_single.pose.orientation.x = 
        pose_covariance = [10000]*36
        pose_covariance[0] = obj.sigma[0, 0]
        pose_covariance[1] = obj.sigma[0, 1]
        pose_covariance[6] = obj.sigma[1, 0]
        pose_covariance[7] = obj.sigma[1, 1]
        msg_single.pose.pose.covariance = pose_covariance
        msg_class = ObjectClass()
        msg_class.classid = obj.classid
        msg_class.comments = obj.classname
        msg_class.score = obj.lost_count / 5.0
        msg_single.classes.append(msg_class)
        msg_single.twist.twist.linear.x = obj.velo_x
        msg_single.twist.twist.linear.y = obj.velo_y
        msg_single.twist.twist.linear.z = 0
        msg_single.twist.twist.angular.x = 0
        msg_single.twist.twist.angular.y = 0
        msg_single.twist.twist.angular.z = 0
        twist_covariance = [10000]*36
        twist_covariance[0] = obj.sigma[2, 2]
        twist_covariance[1] = obj.sigma[2, 3]
        twist_covariance[6] = obj.sigma[3, 2]
        twist_covariance[7] = obj.sigma[3, 3]
        msg_single.twist.covariance = twist_covariance

        return msg_single
    
    def gen_current_frame_list(self, detections):
        self.vehicle_list_cur = []
        self.pedestrian_list_cur = []

        if len(detections) < 1:
            return

        if self.x is None or self.y is None or self.yaw is None:
            return
        for detect in detections:
            x = detect.bbox.pose.x
            y = detect.bbox.pose.y
            classid = detect.classes[0].classid
            classname = detect.classes[0].classname
            
            self.process_single_detect(x, y, self.x, self.y, self.yaw, classid, classname)
            

        ## print the perception poses from targets 
        # print("perception\n")
        # print("vehicle\n")
        # for target_vehicle in self.vehicle_list_cur:
        #     t_loc_array = np.array([target_vehicle.location.x, target_vehicle.location.y])
        #     print(t_loc_array)
        # print("pedestrian\n")
        # for target_ped in self.pedestrian_list_cur:
        #     t_loc_array = np.array([target_ped.location.x, target_ped.location.y])
        #     print(t_loc_array)

    # def process_single_detect(self, x, y, x_self, y_self, yaw_self, classid, classname=None):
    #     xy_local = np.array([x, y]).reshape((2, 1))
    #     yaw = math.radians(yaw_self)
    #     # print(yaw)
    #     R_mat = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    #     xy_self = np.array([x_self, y_self]).reshape((2, 1))
    #     xy_global = R_mat.dot(xy_local) + xy_self
    #     # print(xy_local.flatten())
    #     # print(xy_global.flatten())
    #     # print('detect', detect[2])
    #     if classname == 'car' or classname == 'truck' or classname == 'bus':
    #         obj = SimpleObj(xy_global, xy_local, classname, classid)
    #         self.vehicle_list_cur.append(obj)

    #     elif classname == 'person' or classname == 'bicycle' or classname == 'motorcycle':
    #         obj = SimpleObj(xy_global, xy_local, classname, classid)
    #         self.pedestrian_list_cur.append(obj)


    # def run_step(self, input_data, pos_self, yaw_self):
    #     detections = self.process_input_data(input_data)
    #     # print the perception poses from targets 
    #     # print("\n perception")
    #     # for target_vehicle in detections:
    #     #     print(target_vehicle)

    #     self.gen_current_frame_list(detections, pos_self, yaw_self)

    #     # print("\n needed perception")
    #     # print("vehicle")
    #     # for target_vehicle in self.vehicle_list_cur:
    #     #     # print(type(target_vehicle))
    #     #     # t_loc = target_vehicle.location
    #     #     # t_loc_array = np.array([t_loc.x, t_loc.y])
    #     #     t_loc_array = np.array([target_vehicle.location.x, target_vehicle.location.y])
    #     #     print(t_loc_array)
    #     # print("pedestrian")
    #     # for target_ped in self.pedestrian_list_cur:
    #     #     t_loc_array = np.array([target_ped.location.x, target_ped.location.y])
    #     #     print(t_loc_array)

    #     self.vehicle_list.updatelist(self.vehicle_list_cur)
    #     self.pedestrian_list.updatelist(self.pedestrian_list_cur)

    #     # print("\n tracked")
    #     # print("vehicle")
    #     # for target_vehicle in self.vehicle_list:
    #     #     # print(type(target_vehicle))
    #     #     # t_loc = target_vehicle.location
    #     #     # t_loc_array = np.array([t_loc.x, t_loc.y])
    #     #     t_loc_array = np.array([target_vehicle.location.x, target_vehicle.location.y])
    #     #     print(t_loc_array)
    #     # print("pedestrian")
    #     # for target_ped in self.pedestrian_list:
    #     #     t_loc_array = np.array([target_ped.location.x, target_ped.location.y])
    #     #     print(t_loc_array)