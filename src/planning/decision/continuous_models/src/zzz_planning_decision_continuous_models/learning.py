
import socket
import msgpack
import rospy
import math
import numpy as np

from zzz_cognition_msgs.msg import MapState
from zzz_planning_decision_continuous_models.Werling_trajectory import Werling
from zzz_driver_msgs.utils import get_speed
from carla import Location, Rotation, Transform
from zzz_common.geometry import dense_polyline2d
from zzz_common.kinematics import get_frenet_state



class RLSPlanner(object):
    """
    Parameter:
        mode: ZZZ TCP connection mode (client/server)
    """
    def __init__(self, openai_server="127.0.0.1", port=2333, mode="client", recv_buffer=4096):
        self._dynamic_map = None
        self._socket_connected = False
        self._rule_based_trajectory_model_instance = Werling()
        self._buffer_size = recv_buffer
        self._collision_signal = False
        self._collision_times = 0
    
        if mode == "client":
            rospy.loginfo("Connecting to RL server...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((openai_server, port))
            self._socket_connected = True
            rospy.loginfo("Connected...")
        else:
            # TODO: Implement server mode to make multiple connection to this node.
            #     In this mode, only rule based action is returned to system
            raise NotImplementedError("Server mode is still wating to be implemented.") 
        
    def speed_update(self, trajectory, dynamic_map):
        return 10   

    def something_in_lane(self):
        self._rule_based_trajectory_model_instance.last_trajectory_array = []
        self._rule_based_trajectory_model_instance.last_trajectory = []
        self._rule_based_trajectory_model_instance.last_trajectory_array_rule = []
        self._rule_based_trajectory_model_instance.last_trajectory_rule = []
        self._collision_signal = False

        # send done to OPENAI
        collision = False
        leave_current_mmap = 1
        sent_RL_msg = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        sent_RL_msg.append(collision)
        sent_RL_msg.append(leave_current_mmap)
        sent_RL_msg.append(5.0) # RL.point
        sent_RL_msg.append(0.0)
        # print("-----------------------------",sent_RL_msg)
        self.sock.sendall(msgpack.packb(sent_RL_msg))

        try:
            RLS_action = msgpack.unpackb(self.sock.recv(self._buffer_size))
        except:
            pass

        return None

    def trajectory_update(self, dynamic_map):
        now1 = rospy.get_rostime()
        self._dynamic_map = dynamic_map
        self._rule_based_trajectory_model_instance.update_dynamic_map(dynamic_map)        
        #found closest obs
        closest_obs = self.found_closest_obstacles(3, self._dynamic_map)
        RL_state = self.wrap_state(closest_obs)
        sent_RL_msg = RL_state

        collision = int(self._collision_signal)
        self._collision_signal = False
        leave_current_mmap = 0
        sent_RL_msg.append(collision)
        sent_RL_msg.append(leave_current_mmap)

        # for teaching
        trajectory_rule, desired_speed_rule, generated_trajectory_rule = self._rule_based_trajectory_model_instance.trajectory_update(dynamic_map, closest_obs)
        delta_T = 0.75
        RLpoint = self.get_RL_point_from_trajectory(generated_trajectory_rule , delta_T)
        sent_RL_msg.append(RLpoint.location.x)
        sent_RL_msg.append(RLpoint.location.y)
        # sent_RL_msg.append(5.0) # RL.point
        # sent_RL_msg.append(0.0)
        now2 = rospy.get_rostime()
        print("-----------------------------rule-based time consume",now2.to_sec() - now1.to_sec())
        print("-----------------------------",sent_RL_msg)
        self.sock.sendall(msgpack.packb(sent_RL_msg))

        try:
            # received RL action and plan a RL trajectory
            RLS_action = msgpack.unpackb(self.sock.recv(self._buffer_size))
            rospy.logdebug("received action:%f, %f", RLS_action[0], RLS_action[1])
            now3 = rospy.get_rostime()
            print("-----------------------------socket time consume",now3.to_sec() - now2.to_sec())
            RLS_action[0] = RLS_action[0] + 15.0
            print("-+++++++++++++++++++++++++++++++++++++++++++befor-generated RL trajectory")
            trajectory, desired_speed, generated_trajectory = self._rule_based_trajectory_model_instance.trajectory_update_withRL(dynamic_map, RLS_action)
            print("-+++++++++++++++++++++++++++++++++++++++++++-generated RL trajectory")
            now4 = rospy.get_rostime()
            print("-----------------------------rl planning time consume",now4.to_sec() - now3.to_sec())
            return trajectory, desired_speed 

        except:
            rospy.logerr("Continous RLS Model cannot receive an action")
            return [], 0
            
            
    def wrap_state(self, closest_obs):

        # Set State space = 4+4*obs_num

        # ego state: ego_x(0), ego_y(1), ego_vx(2), ego_vy(3)    
            # obstacle 0 : x0(4), y0(5), vx0(6), vy0(7)
            # obstacle 1 : x0(8), y0(9), vx0(10), vy0(11)
            # obstacle 2 : x0(12), y0(13), vx0(14), vy0(15)

        reference_path_from_map = self._dynamic_map.jmap.reference_path.map_lane.central_path_points
        
        ref_path_ori = self.convert_path_to_ndarray(reference_path_from_map)
        ref_path = dense_polyline2d(ref_path_ori, 1)
        ref_path_tangets = np.zeros(len(ref_path))

        ffstate = get_frenet_state(self._dynamic_map.ego_state, ref_path, ref_path_tangets)

        state = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      
        state[0] = ffstate.s
        state[1] = -ffstate.d
        state[2] = ffstate.vs
        state[3] = ffstate.vd

        
        i = 0
        for obs in closest_obs: 
            if i < 3:               
                state[(i+1)*4+0] = obs[5]
                state[(i+1)*4+1] = obs[6]
                state[(i+1)*4+2] = obs[7]
                state[(i+1)*4+3] = obs[8]

                i = i+1
            else:
                break

        return state

    def convert_path_to_ndarray(self, path):
        point_list = [(point.position.x, point.position.y) for point in path]
        return np.array(point_list)

    def found_closest_obstacles(self, num, dynamic_map):
        closest_obs = []
        obs_tuples = []
        reference_path_from_map = self._dynamic_map.jmap.reference_path.map_lane.central_path_points
        
        ref_path_ori = self.convert_path_to_ndarray(reference_path_from_map)
        ref_path = dense_polyline2d(ref_path_ori, 1)
        ref_path_tangets = np.zeros(len(ref_path))
        
        for obs in self._dynamic_map.jmap.obstacles: 
            # calculate distance
            p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x , self._dynamic_map.ego_state.pose.pose.position.y])
            p2 = np.array([obs.state.pose.pose.position.x , obs.state.pose.pose.position.y])
            p3 = p2 - p1
            p4 = math.hypot(p3[0],p3[1])

            # transfer to frenet
            obs_ffstate = get_frenet_state(obs.state, ref_path, ref_path_tangets)
            one_obs = (obs.state.pose.pose.position.x , obs.state.pose.pose.position.y , obs.state.twist.twist.linear.x , obs.state.twist.twist.linear.y , p4 , obs_ffstate.s , -obs_ffstate.d , obs_ffstate.vs , obs_ffstate.vd)
            # if obs.ffstate.s > 0.01:
            obs_tuples.append(one_obs)
            print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++closestone_obs",one_obs[4])

        
        sorted_obs = sorted(obs_tuples, key=lambda obs: obs[4])   # sort by distance
        i = 0
        for obs in sorted_obs:
            if i < num:
                closest_obs.append(obs)
                i = i + 1
            else:
                break
        
        return closest_obs


    def RL_model_matching(self):
        pass

    def get_RL_point_from_trajectory(self, generated_trajectory , delta_T):
        RLpoint = Transform()
   
        if generated_trajectory is not None:
            RLpoint.location.x = generated_trajectory.s[5] #only works when DT param of werling is 0.15
            RLpoint.location.y = generated_trajectory.d[5]
        else:
            RLpoint.location.x = 5
            RLpoint.location.y = 0           
        return RLpoint

    def get_decision_from_continuous_action(self, action, delta_T, acc = 2,  hard_brake = 4):
        # for a straight line when RL return x,y but not s,d 
        print("__________________________________________action[0]",action[0])
        print("__________________________________________action[1]",action[1])

        action[0], action[1] = self.transfer_to_world(action[0],action[1], self._dynamic_map)
        print("__________________________________________afteraction[0]",action[0])
        print("__________________________________________afteraction[1]",action[1])
        trajectoryx = [action[0], (action[0]+self._dynamic_map.ego_state.pose.pose.position.x)/2, self._dynamic_map.ego_state.pose.pose.position.x]
        trajectoryy = [-action[1], (-action[1]+self._dynamic_map.ego_state.pose.pose.position.y)/2, self._dynamic_map.ego_state.pose.pose.position.y]

        trajectory = np.c_[trajectoryx , trajectoryy]
        trajectory_array = dense_polyline2d(trajectory,1)

        p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x,self._dynamic_map.ego_state.pose.pose.position.y])
        p2 = np.array([action[0],action[1]])
        p3 = p2 - p1
        p4 = math.hypot(p3[0],p3[1])

        desired_speed = p4 / delta_T

        return trajectory_array, desired_speed

    def transfer_to_intersection(self, pointx, pointy, dynamic_map):
        print("____________________transfer_to_intersection",dynamic_map.ego_state.pose.pose.position.x,dynamic_map.ego_state.pose.pose.position.y)

        if 5.0 < dynamic_map.ego_state.pose.pose.position.x < 55.0 and -25.0 < -dynamic_map.ego_state.pose.pose.position.y < 25.0:
            pointx = pointx - 5.0 - 25.0
            pointy = pointy + 25.0 - 25.0
            print("____________________intersection1")
        elif 5.0 < dynamic_map.ego_state.pose.pose.position.x < 55.0 and -115.0 < -dynamic_map.ego_state.pose.pose.position.y < -65.0:
            pointx = pointx - 5.0 - 25.0
            pointy = pointy + 115.0 - 25.0
            print("____________________intersection2")

        elif -75.0 < dynamic_map.ego_state.pose.pose.position.x < -25.0 and -25.0 < -dynamic_map.ego_state.pose.pose.position.y < 25.0:
            pointx = pointx + 75.0 - 25.0
            pointy = pointy + 25.0 - 25.0
            print("____________________intersection3")

        elif -75.0 < dynamic_map.ego_state.pose.pose.position.x < -25.0 and -115.0 < -dynamic_map.ego_state.pose.pose.position.y < -65.0:
            pointx = pointx + 75.0 - 25.0
            pointy = pointy + 115.0 - 25.0
            print("____________________intersection4")


        return pointx, pointy

    def transfer_to_world(self, pointx, pointy, dynamic_map):
        print("____________________transfer_to_world",dynamic_map.ego_state.pose.pose.position.x,dynamic_map.ego_state.pose.pose.position.y)

        if 5.0 < dynamic_map.ego_state.pose.pose.position.x < 55.0 and -25.0 < -dynamic_map.ego_state.pose.pose.position.y < 25.0:
            pointx = pointx + 5.0 + 25.0
            pointy = pointy - 25.0 + 25.0
            print("____________________intersection1")

        elif 5.0 < dynamic_map.ego_state.pose.pose.position.x < 55.0 and -115.0 < -dynamic_map.ego_state.pose.pose.position.y < -65.0:
            pointx = pointx + 5.0 + 25.0
            pointy = pointy - 115.0 + 25.0
            print("____________________intersection2")

        elif -75.0 < dynamic_map.ego_state.pose.pose.position.x < -25.0 and -25.0 < -dynamic_map.ego_state.pose.pose.position.y < 25.0:
            pointx = pointx - 75.0 + 25.0
            pointy = pointy - 25.0 + 25.0
            print("____________________intersection3")

        elif -75.0 < dynamic_map.ego_state.pose.pose.position.x < -25.0 and -115.0 < -dynamic_map.ego_state.pose.pose.position.y < -65.0:
            pointx = pointx - 75.0 + 25.0
            pointy = pointy - 115.0 + 25.0
            print("____________________intersection4")


        return pointx, pointy