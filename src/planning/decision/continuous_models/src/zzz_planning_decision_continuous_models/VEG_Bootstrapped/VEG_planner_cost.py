
import socket
import msgpack
import rospy
import math
import numpy as np
import time

from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed
from carla import Location, Rotation, Transform
from zzz_common.geometry import dense_polyline2d
from zzz_common.kinematics import get_frenet_state

from zzz_planning_msgs.msg import DecisionTrajectory
from zzz_planning_decision_continuous_models.VEG_Bootstrapped.Werling_planner_RL_cost import Werling
from zzz_planning_decision_continuous_models.common import rviz_display, convert_ndarray_to_pathmsg, convert_path_to_ndarray
from zzz_planning_decision_continuous_models.predict import predict

# PARAMETERS
OBSTACLES_CONSIDERED = 3
ACTION_SPACE_SYMMERTY = 15/3.6 



class VEG_Planner_cost(object):
    """
    Parameter:
        mode: ZZZ TCP connection mode (client/server)
    """
    def __init__(self, openai_server="127.0.0.1", port=2333, mode="client", recv_buffer=4096, socket_time_out = 1000000):
        self._dynamic_map = None
        self._socket_connected = False
        self._rule_based_trajectory_model_instance = Werling()
        self._buffer_size = recv_buffer
        self._collision_signal = False
        self._collision_times = 0
        self._has_clear_buff = True
        self.has_send_colli_to_gym = False

        self.reference_path = None
        self.ref_path = None
        self.ref_path_tangets = None

        self.rivz_element = rviz_display()
        self.kick_in_signal = None

    
        if mode == "client":
            rospy.loginfo("Connecting to RL server...")
            socket.setdefaulttimeout(socket_time_out) # Set time out
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(socket_time_out) # Set time out

            self.sock.connect((openai_server, port))
            self._socket_connected = True
            
            rospy.loginfo("Connected...")
        else:
            # TODO: Implement server mode to make multiple connection to this node.
            #     In this mode, only rule based action is returned to system
            raise NotImplementedError("Server mode is still wating to be implemented.") 
        
    def initialize(self, dynamic_map):
        try:
            if self.ref_path_tangets is None:
                self.reference_path = dynamic_map.jmap.reference_path.map_lane.central_path_points
                ref_path_ori = convert_path_to_ndarray(self.reference_path)
                self.ref_path = dense_polyline2d(ref_path_ori, 2)
                self.ref_path_tangets = np.zeros(len(self.ref_path))

            return True
        except:
            print("------> VEG: Initialize fail ")
            return False

    def clear_buff(self, dynamic_map):
        # send done to OPENAI
        if self._has_clear_buff == False:
            ego_x = dynamic_map.ego_state.pose.pose.position.x
            ego_y = dynamic_map.ego_state.pose.pose.position.y
            # if self._collision_signal == True:
            #     leave_current_mmap = 2

            #     sent_RL_msg = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # ego and obs state
            #     sent_RL_msg.append(self._collision_signal)
            #     sent_RL_msg.append(leave_current_mmap)

            #     print("Send State Msg:",sent_RL_msg)
            #     self.sock.sendall(msgpack.packb(sent_RL_msg))

            # if math.pow((ego_x+10),2) + math.pow((ego_y-94),2) < 64:  # restart point - Town05
            if self._collision_signal == True:
                # sent_RL_msg = [2 for i in range((OBSTACLES_CONSIDERED + 1) * 4)] # ego and obs state
                sent_RL_msg = self.wrap_state()
                sent_RL_msg.append(self._collision_signal)
                sent_RL_msg.append(0)
            
            elif math.pow((ego_x - 136),2) + math.pow((ego_y + 220),2) < 64:  # restart point - Town02
                leave_current_mmap = 2
                # sent_RL_msg = [1 for i in range((OBSTACLES_CONSIDERED + 1) * 4)] # ego and obs state
                sent_RL_msg = self.wrap_state()
                sent_RL_msg.append(self._collision_signal)
                sent_RL_msg.append(leave_current_mmap)
            else:  
                leave_current_mmap = 1
                sent_RL_msg = self.wrap_state()
                # sent_RL_msg = [0 for i in range((OBSTACLES_CONSIDERED + 1) * 4)] # ego and obs state
                sent_RL_msg.append(self._collision_signal)
                sent_RL_msg.append(leave_current_mmap)

            print("Send State Msg:",sent_RL_msg)
            self.sock.sendall(msgpack.packb(sent_RL_msg))
            self._has_clear_buff = True
            self._rule_based_trajectory_model_instance.clear_buff(dynamic_map)
            self.reference_path = None
            self.ref_path = None
            self.ref_path_tangets = None
            self.has_send_colli_to_gym = False
        self._collision_signal = False

        return None

    def trajectory_update(self, dynamic_map):

        if self.initialize(dynamic_map):

            time1 = time.time()

            self._has_clear_buff = False
            self._dynamic_map = dynamic_map

            # rule-based planner
            rule_trajectory_msg = self._rule_based_trajectory_model_instance.trajectory_update(dynamic_map)

            # wrap states
            # if self._collision_signal == True and self.has_send_colli_to_gym == True: # Only send collision state to gym once
            #     return rule_trajectory_msg
            
            sent_RL_msg = self.wrap_state()
            sent_RL_msg.append(int(0)) # This is always 0, the collision signal will be sent by clear_buff()
            sent_RL_msg.append(0)

            print("Send State Msg:",sent_RL_msg)
            self.sock.sendall(msgpack.packb(sent_RL_msg))

            # received RL action and plan a RL trajectory
            # if self._collision_signal == False and self.has_send_colli_to_gym == False: # Only received action at no collision time
            try:
                received_msg = msgpack.unpackb(self.sock.recv(self._buffer_size))
                rls_action = received_msg
                print("Received Action:",rls_action)
                time2 = time.time()
                time.sleep(0.1 - (time2 - time1) / 1000) #FIXME: for delay in time 
                if rls_action == 0:
                    print("----> VEG: Rule-based planning")           
                    return rule_trajectory_msg
                else:
                    return self._rule_based_trajectory_model_instance.trajectory_update_RLS(dynamic_map, rls_action)
            except:
                print("----> VEG: Fail to Received ActionW")
                return None      
        else:
            return None   
            
    def wrap_state(self, leave_current_mmap = 0):
        # ego state: ego_x(0), ego_y(1), ego_vx(2), ego_vy(3)    
        # obstacle 0 : x0(4), y0(5), vx0(6), vy0(7)
        # obstacle 1 : x0(8), y0(9), vx0(10), vy0(11)
        # obstacle 2 : x0(12), y0(13), vx0(14), vy0(15)
        state = [0 for i in range((OBSTACLES_CONSIDERED + 1) * 4)]

        if self._dynamic_map is not None:
            # ego state
            ego_ffstate = get_frenet_state(self._dynamic_map.ego_state, self.ref_path, self.ref_path_tangets)
            state[0] = ego_ffstate.s 
            state[1] = -ego_ffstate.d
            state[2] = ego_ffstate.vs
            state[3] = ego_ffstate.vd

            # obs state
            closest_obs = []
            closest_obs = self.found_closest_obstacles_t_intersection(OBSTACLES_CONSIDERED, self._dynamic_map)
            i = 0
            for obs in closest_obs: 
                if i < OBSTACLES_CONSIDERED:
                    if obs[5] != 0:              
                        state[(i+1)*4+0] = obs[5] #- ego_ffstate.s 
                    if obs[6] != 0:  
                        state[(i+1)*4+1] = obs[6] #+ ego_ffstate.d
                    state[(i+1)*4+2] = obs[7]
                    state[(i+1)*4+3] = obs[8]
                    i = i+1
                else:
                    break
        
        return state

    def found_closest_obstacles(self, num, dynamic_map):
        closest_obs = []
        obs_tuples = []
        
        for obs in self._dynamic_map.jmap.obstacles: 
            
            # Calculate distance
            p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x , self._dynamic_map.ego_state.pose.pose.position.y])
            p2 = np.array([obs.state.pose.pose.position.x , obs.state.pose.pose.position.y])
            p3 = p2 - p1
            p4 = math.hypot(p3[0],p3[1])

            # Obstacles too far
            if p4 > 50:
                continue

            # Transfer to frenet
            obs_ffstate = get_frenet_state(obs.state, self.ref_path, self.ref_path_tangets)
            one_obs = (obs.state.pose.pose.position.x , obs.state.pose.pose.position.y , obs.state.twist.twist.linear.x ,
                     obs.state.twist.twist.linear.y , p4 , obs_ffstate.s , -obs_ffstate.d , obs_ffstate.vs, obs_ffstate.vd, 
                     obs.state.accel.accel.linear.x, obs.state.accel.accel.linear.y)
            obs_tuples.append(one_obs)

        # Sort by distance
        sorted_obs = sorted(obs_tuples, key=lambda obs: obs[4])   
        i = 0
        for obs in sorted_obs:
            if i < num:
                closest_obs.append(obs)
                i = i + 1
            else:
                break
        
        return closest_obs

    def found_closest_obstacles_four_direction(self, num, dynamic_map):
        obs_tuples = []
        ego_ffstate = get_frenet_state(self._dynamic_map.ego_state, self.ref_path, self.ref_path_tangets)
        for obs in self._dynamic_map.jmap.obstacles: 
            
            # Calculate distance
            p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x , self._dynamic_map.ego_state.pose.pose.position.y])
            p2 = np.array([obs.state.pose.pose.position.x , obs.state.pose.pose.position.y])
            p3 = p2 - p1
            p4 = math.hypot(p3[0],p3[1])
            
            # Obstacles too far
            if p4 > 50:
                continue
            
            obs_ffstate = get_frenet_state(obs.state, self.ref_path, self.ref_path_tangets)
            p5 = np.array([ego_ffstate.s , -ego_ffstate.d])
            p6 = np.array([obs_ffstate.s , -obs_ffstate.d])
            p7 = p6 - p5  

            # Transfer to frenet
            one_obs = (obs.state.pose.pose.position.x , obs.state.pose.pose.position.y , obs.state.twist.twist.linear.x ,
                     obs.state.twist.twist.linear.y , p4 , obs_ffstate.s , -obs_ffstate.d , obs_ffstate.vs, obs_ffstate.vd, 
                     obs.state.accel.accel.linear.x, obs.state.accel.accel.linear.y, p7)
            obs_tuples.append(one_obs)
        
        closest_obs = []
        fake_obs = [0 for i in range(11)]  #len(one_obs)
        for i in range(0,4,1): # 4 directions
            closest_obs.append(fake_obs)
        
        # Sort by distance
        sorted_obs = sorted(obs_tuples, key=lambda obs: obs[4])   

        put_1st = False
        put_2nd = False
        put_3rd = False
        put_4th = False
        for obs in sorted_obs:
            if obs[11][0] > 0 and obs[11][1] > 0 and put_1st == False:
                closest_obs[0] = obs
                put_1st = True
            if obs[11][0] > 0 and obs[11][1] < 0 and put_2nd == False:
                closest_obs[1] = obs
                put_2nd = True
            if obs[11][0] < 0 and obs[11][1] > 0 and put_3rd == False:
                closest_obs[2] = obs
                put_3rd = True
            if obs[11][0] < 0 and obs[11][1] < 0 and put_4th == False:
                closest_obs[3] = obs
                put_4th = True
            else:
                continue

        return closest_obs
        
    def found_closest_obstacles_t_intersection(self, num, dynamic_map):
        obs_tuples = []
        ego_ffstate = get_frenet_state(self._dynamic_map.ego_state, self.ref_path, self.ref_path_tangets)
        for obs in self._dynamic_map.jmap.obstacles: 
            
            # Calculate distance
            p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x , self._dynamic_map.ego_state.pose.pose.position.y])
            p2 = np.array([obs.state.pose.pose.position.x , obs.state.pose.pose.position.y])
            p3 = p2 - p1
            p4 = math.hypot(p3[0],p3[1])
            
            # Obstacles too far
            if p4 > 30:
                continue
            
            obs_ffstate = get_frenet_state(obs.state, self.ref_path, self.ref_path_tangets)
            p5 = np.array([ego_ffstate.s , -ego_ffstate.d])
            p6 = np.array([obs_ffstate.s , -obs_ffstate.d])
            p7 = p6 - p5  

            # Transfer to frenet
            one_obs = (obs.state.pose.pose.position.x , obs.state.pose.pose.position.y , obs.state.twist.twist.linear.x ,
                     obs.state.twist.twist.linear.y , p4 , obs_ffstate.s , -obs_ffstate.d , obs_ffstate.vs, obs_ffstate.vd, 
                     obs.state.accel.accel.linear.x, obs.state.accel.accel.linear.y, p7)
            obs_tuples.append(one_obs)
        
        closest_obs = []
        fake_obs = [0 for i in range(11)]  #len(one_obs)
        for i in range(0,3,1): # 3 obs
            closest_obs.append(fake_obs)
        
        # Sort by distance
        sorted_obs = sorted(obs_tuples, key=lambda obs: obs[4])   

        put_1st = False
        put_2nd = False
        put_3rd = False
        for obs in sorted_obs:
            if obs[0] < 134 and obs[1] < -190 and obs[1] > -194 and obs[2] > 0 and math.fabs(obs[3]) < 0.5 and put_1st == False:
                closest_obs[0] = obs
                put_1st = True
            if obs[1] > -205 and obs[1] < -185 and obs[2] < 0.5 and (obs[1] > self._dynamic_map.ego_state.pose.pose.position.y - 3 or obs[1] > -190) \
                    and obs[0] < self._dynamic_map.ego_state.pose.pose.position.x and put_2nd == False:
                closest_obs[1] = obs
                put_2nd = True
            if obs[1] > -205 and obs[1] < -185 and obs[2] < 0.5 and (obs[1] > self._dynamic_map.ego_state.pose.pose.position.y - 3 or obs[1] > -190) \
                     and obs[0] > self._dynamic_map.ego_state.pose.pose.position.x and put_3rd == False:
                closest_obs[2] = obs
                put_3rd = True
            else:
                continue
        return closest_obs