
import socket
import msgpack
import rospy

from zzz_cognition_msgs.msg import MapState
from zzz_planning_decision_lane_models.longitudinal import IDM
from zzz_planning_decision_lane_models.lateral import LaneUtility
from zzz_driver_msgs.utils import get_speed

class RLSDecision(object):
    """
    Parameter:
        mode: ZZZ TCP connection mode (client/server)
    """
    def __init__(self, openai_server="127.0.0.1", port=2345, mode="client", recv_buffer=4096):
        self._dynamic_map = None
        self._socket_connected = False
        self._rule_based_longitudinal_model_instance = IDM()
        self._rule_based_lateral_model_instance = LaneUtility(self._rule_based_longitudinal_model_instance)
        self._buffer_size = recv_buffer
        self._collision_signal = False
        self._collision_times = 0

        self.inside_lane = None
        self.outside_lane = None


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

    def lateral_decision(self, dynamic_map):



        self._dynamic_map = dynamic_map
        self._rule_based_longitudinal_model_instance.update_dynamic_map(dynamic_map)
        # return -1, self._rule_based_longitudinal_model_instance.longitudinal_speed(-1)


        # Following reference path in junction # TODO(Zhong):should be in cognition part
        if dynamic_map.model == MapState.MODEL_JUNCTION_MAP or dynamic_map.mmap.target_lane_index == -1:
            # send done to OPENAI
            collision = int(self._collision_signal)
            self._collision_signal = False
            leave_current_mmap = 1
            sent_RL_msg = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            sent_RL_msg.append(collision)
            sent_RL_msg.append(leave_current_mmap)
            self.sock.sendall(msgpack.packb(sent_RL_msg))

            return -1, self._rule_based_longitudinal_model_instance.longitudinal_speed(-1)

        RL_state = self.wrap_state()
        rospy.logdebug("sending state: ego_y:%.1f, ego_v:%.1f", RL_state[0],RL_state[1])
        sent_RL_msg = RL_state

        collision = int(self._collision_signal)
        self._collision_signal = False
        leave_current_mmap = 0
        sent_RL_msg.append(collision)
        sent_RL_msg.append(leave_current_mmap)

        self.sock.sendall(msgpack.packb(sent_RL_msg))
        RLS_action = msgpack.unpackb(self.sock.recv(self._buffer_size))
        # RLS_action = 0
        rospy.logdebug("received action:%d", RLS_action)

        return self.get_decision_from_discrete_action(RLS_action)

    def wrap_state(self):

        
        self.inside_lane = 1
        self.outside_lane = 0

        state = []
        
        state.append(self._dynamic_map.ego_state.ffstate.s)
        state.append(self._dynamic_map.ego_state.ffstate.d)
        state.append(self._dynamic_map.ego_state.ffstate.vs)
        state.append(self._dynamic_map.ego_state.ffstate.vd)

        
        # TODO(Tao): find the right information
        for i,lane in enumerate(self._dynamic_map.mmap.lanes):
            self.get_state_from_lane_front(lane,i,state)
        for i,lane in enumerate(self._dynamic_map.mmap.lanes):
            self.get_state_from_lane_behind(lane,i,state)

        return state

    def RL_model_matching(self):
        pass

    def get_decision_from_discrete_action(self, action, acc = 2, decision_dt = 0.75, hard_brake = 10):

        # TODO(Zhong): check if action is reasonable

        # always rule based for test!!!!!!!!!!!!!!!!!!!!
        return self._rule_based_lateral_model_instance.lateral_decision(self._dynamic_map)


        # Rule-based action
        if action == 0:
            return self._rule_based_lateral_model_instance.lateral_decision(self._dynamic_map)

        current_speed = get_speed(self._dynamic_map.ego_state)
        ego_y = self._dynamic_map.mmap.ego_lane_index

        # Hard-brake action
        if action == 1:
            return ego_y, current_speed - hard_brake * decision_dt

        # ego lane action
        if action == 2:
            return self.outside_lane, current_speed

        if action == 3:
            return self.inside_lane, current_speed

        if action == 4:
            return self.outside_lane, current_speed + acc * decision_dt

        if action == 5:
            return self.inside_lane, current_speed + acc * decision_dt

        if action == 6:
            return self.outside_lane, current_speed - acc * decision_dt

        if action == 7:
            return self.inside_lane, current_speed + acc * decision_dt



    def get_state_from_lane_front(self,lane,lane_index,state,
                                range_x = 100,
                                range_vx = 50/3.6
                                ):
        if len(lane.front_vehicles) > 0:
            fv = lane.front_vehicles[0]
            fv_id = fv.uid
            fv_s = fv.ffstate.s
            fv_d = fv.ffstate.d
            fv_vs = fv.ffstate.vs
            fv_vd = fv.ffstate.vd

            # fv_vy = fv.mmap_vy # FIXME(zhong): should consider lane change speed
        else:
            fv_s = 0
            fv_d = 0
            fv_vs = 0
            fv_vd = 0

        state.append(fv_s)
        state.append(fv_d)
        state.append(fv_vs)
        state.append(fv_vd)

        # rospy.logdebug("lane %d: (%d)fv_x:%.1f, fv_y:%.1f, fv_vx:%.1f,||(%d)rv_x:%.1f, rv_y:%.1f, rv_vx:%.1f", lane_index,
        #                                 fv_id,fv_x,fv_y,fv_vx,rv_id,rv_x,rv_y,rv_vx)


    def get_state_from_lane_behind(self,lane,lane_index,state,
                                range_x = 100,
                                range_vx = 50/3.6
                                ):

        if len(lane.rear_vehicles) > 0:
            rv = lane.rear_vehicles[0]
            rv_id = rv.uid
            rv_s = rv.ffstate.s
            rv_d = rv.ffstate.d
            rv_vs = rv.ffstate.vs
            rv_vd = rv.ffstate.vd
        else:
            rv_s = 0
            rv_d = 0
            rv_vs = 0
            rv_vd = 0

        state.append(rv_s)
        state.append(rv_d)
        state.append(rv_vs)
        state.append(rv_vd)
        # rospy.logdebug("lane %d: (%d)fv_x:%.1f, fv_y:%.1f, fv_vx:%.1f,||(%d)rv_x:%.1f, rv_y:%.1f, rv_vx:%.1f", lane_index,
        #                                 fv_id,fv_x,fv_y,fv_vx,rv_id,rv_x,rv_y,rv_vx)
