
import numpy as np
import math

import sys
from yolov3.YoloDetect import YoloDetect

# import carla
# from .SurroundingObjects import Surrounding_pedestrian, Surrounding_vehicle

# class PercpVeh():
#     def __init__(self, id=None, x=None, y=None, vx=None, vy=None, simple_veh=None):

#         self.last_location = None #carla.Location(x=0, y=0, z=0)

#         self.matched = True
#         self.lost_count = 0

#         self.mot_noise = np.diag([1, 1, 3, 3])
#         self.obs_noise = np.diag([0.5, 0.5])
#         self.sigma = np.diag([1,1,50,50])

#         assert id is not None, "id should be given"
#         self.id = id
#         # self.time = time

#         if simple_veh is not None:
#             self.state = np.array([simple_veh.location.x, simple_veh.location.y, 5, 5])[:,np.newaxis]
#         elif x is None:
#             self.state = None
#         else:
#             assert x is not None and y is not None, "Info not complete"
#             if vx is None and vy is None:
#                 self.state = np.array([x, y, 0, 0])[:,np.newaxis]
#             else:
#                 assert vx is not None and vy is not None, "velocity is not complete"
#                 self.state = np.array([x, y, vx, vy])[:,np.newaxis]

#     def update(self, simple_veh=None):
#         if simple_veh is not None:
#             x = simple_veh.location.x
#             y = simple_veh.location.y
#             self.filter(x, y)
#             self.matched = True
#             self.lost_count = 0
#         else:
#             self.matched = False
#             self.lost_count += 1
    
#     def filter(self, x, y):
        
#         self.last_location = self.location
#         # dt = time - self.time
#         dt = 0.05
#         # print(dt)
#         # self.time = time
#         A = np.array([
#             [1, 0, dt, 0],
#             [0, 1, 0, dt],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#         ])
#         C = np.array([
#             [1, 0, 0, 0],
#             [0, 1, 0, 0]
#         ])
#         self.state = A.dot(self.state)
#         self.sigma = A.dot(self.sigma).dot(A.T) + self.mot_noise

#         z = np.array([[x], [y]]) - C.dot(self.state)
#         S = self.obs_noise + C.dot(self.sigma).dot(C.T)
#         K = self.sigma.dot(C.T).dot(np.linalg.inv(S))
#         self.state += K.dot(z)
#         self.sigma = (np.diag([1]*4) - K.dot(C)).dot(self.sigma)

#     @property
#     def location(self):
#         return carla.Location(x=float(self.state[0,0]), y=float(self.state[1,0]), z=1)

#     @property
#     def speed(self):
#         return 3.6 * math.sqrt(self.state[2,0]**2 + self.state[3,0]**2)

#     @property
#     def speed_direction(self):
#         p_vec = np.array([self.state[2,0], self.state[3,0], 0.0])
#         p_vec = p_vec/np.linalg.norm(p_vec)
#         return p_vec


        
# def evalDist(veh1, veh2):
#     if veh2.matched == False:
#         xd = veh2.location.x - veh1.location.x
#         yd = veh2.location.y - veh1.location.y
#         dist = np.sqrt(xd**2 + yd**2)
#     else:
#         dist = None
#     return dist

# class ObjectList(list):
#     def __init__(self, lost_thresh = 3):
#         self.last_id = 0
#         self.lost_max = lost_thresh
#     def matchto(self, veh_cur):
#         min_dist = None
#         i_best = None
#         for i, veh in enumerate(self):
#             dist = evalDist(veh_cur, veh)
#             if dist is not None:
#                 if min_dist is None:
#                     min_dist = dist
#                     i_best = i
#                 elif dist < min_dist:
#                     min_dist = dist
#                     i_best = i
#         if min_dist is not None:
#             if min_dist < 10:
#                 self[i_best].update(veh_cur)
#                 return True
#         return False

#     def appendto(self, veh_cur):
#         veh = PercpVeh(id=self.last_id, simple_veh=veh_cur)
#         self.last_id += 1
#         self.append(veh)

#     def renewleft(self):
#         if len(self) > 0:
#             to_del_idx = []
#             for i, veh in enumerate(self):
#                 if veh.matched == False:
#                     veh.update()
#                     if veh.lost_count >= self.lost_max:
#                         to_del_idx.append(i)
            
#             to_del_idx_sorted = sorted(to_del_idx, reverse = True)
#             for i in to_del_idx_sorted:
#                 del self[i]
        
                    
    
#     def updatelist(self, veh_list_cur):
#         if len(self)>0:
#             for veh in self:
#                 veh.matched = False
#         if len(veh_list_cur) > 0:
#             if len(self) > 0:
#                 for veh_cur in veh_list_cur:
#                     found = self.matchto(veh_cur)
#                     if not found:
#                         self.appendto(veh_cur)
#             else:
#                 for veh_cur in veh_list_cur:
#                     self.appendto(veh_cur)
#         self.renewleft()
        


        

class CameraProcess(object):
    def __init__(self):
        self.yolo = YoloDetect()
        # self.vehicle_list_cur = []
        # self.pedestrian_list_cur = []

        # self.vehicle_list = ObjectList()
        # self.pedestrian_list = ObjectList()

    def process_input_data(self, array):
        # array = input_data['Left'][1]
        array = np.ascontiguousarray(array[:, :, :3])
        # array = np.ascontiguousarray(array[:, :, ::-1]) # RGB wanted
        # detections = self.yolo.run(array, frame_id=input_data['Left'][0])
        detections = self.yolo.run(array)
        return detections

    # def gen_current_frame_list(self, detections, pos_self, yaw_self):
    #     self.vehicle_list_cur = []
    #     self.pedestrian_list_cur = []

    #     if len(detections) < 1:
    #         return
        
    #     xy_self = np.array([pos_self.x, pos_self.y]).reshape((2, 1))
    #     for detect in detections:
    #         xy_local = np.array([detect[0], detect[1]]).reshape((2, 1))
    #         yaw = math.radians(yaw_self)
    #         # print(yaw)
    #         R_mat = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    #         xy_global = R_mat.dot(xy_local) + xy_self
    #         # print(xy_local.flatten())
    #         # print(xy_global.flatten())
    #         # print('detect', detect[2])
    #         if detect[2] == 'car' or detect[2] == 'truck' or detect[2] == 'bus':
    #             obj = Surrounding_vehicle()
    #             obj.location = carla.Location(x=xy_global[0,0], y=xy_global[1,0],z=0)
    #             obj.speed = 0
    #             obj.speed_direction = np.array([xy_local[0], xy_local[1], 0])
    #             self.vehicle_list_cur.append(obj)
    #         elif detect[2] == 'person' or detect[2] == 'bicycle' or detect[2] == 'motorcycle':
    #             obj = Surrounding_pedestrian()
    #             obj.location = carla.Location(x=xy_global[0,0], y=xy_global[1,0],z=0)
    #             obj.speed = 0
    #             obj.speed_direction = np.array([xy_local[0,0], xy_local[1,0], 0])
    #             self.pedestrian_list_cur.append(obj)

    #     ## print the perception poses from targets 
    #     # print("perception\n")
    #     # print("vehicle\n")
    #     # for target_vehicle in self.vehicle_list_cur:
    #     #     t_loc_array = np.array([target_vehicle.location.x, target_vehicle.location.y])
    #     #     print(t_loc_array)
    #     # print("pedestrian\n")
    #     # for target_ped in self.pedestrian_list_cur:
    #     #     t_loc_array = np.array([target_ped.location.x, target_ped.location.y])
    #     #     print(t_loc_array)

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
        
        



    