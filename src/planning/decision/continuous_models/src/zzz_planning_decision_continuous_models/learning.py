
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

        # Following reference path in junction # TODO(Zhong):should be in cognition part
        if dynamic_map.model == MapState.MODEL_JUNCTION_MAP or dynamic_map.mmap.target_lane_index == -1:
            # send done to OPENAI
            collision = int(self._collision_signal)
            self._collision_signal = False
            leave_current_mmap = 1
            sent_RL_msg = [0,0,100,0,12,-100,0,0,100,1,12,-100,1,0]
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

        # discretize action # TODO(Zhong): continous action
        return self.get_decision_from_discrete_action(RLS_action)

    def wrap_state(self):

        # ego state: ego_y(0), ego_v(1)
        # lane 0 : fv_x(2), fv_y(3), fv_v(4), rv_x(5), rv_y(6), rv_v(7)
        # lane 1 : fv_x(8), fv_y(9), fv_v(10), rv_x(11), rv_y(12), rv_v(13)
        # lane 2:...
        # State space = 2+6*lane_num

        state = []
        ego_y = self._dynamic_map.mmap.ego_ffstate.d
        ego_v = self._dynamic_map.mmap.ego_ffstate.vx
        state.append(ego_y)
        state.append(ego_v)
        for i,lane in enumerate(self._dynamic_map.mmap.lanes):
            self.get_state_from_lane(lane,i,state)

        return state

    def RL_model_matching(self):
        pass

    def get_decision_from_discrete_action(self, action, acc = 2, decision_dt = 0.75, hard_brake = 4):

        # TODO(Zhong): check if action is reasonable

        # Rule-based action
        if action == 0:
            return self._rule_based_lateral_model_instance.lateral_decision(self._dynamic_map)

        current_speed = get_speed(self._dynamic_map.ego_state)
        ego_y = self._dynamic_map.mmap.ego_lane_index

        # Hard-brake action
        if action == 1:
            return ego_y, current_speed-hard_brake*decision_dt

        # ego lane action
        if action == 2:
            return ego_y, current_speed+acc*decision_dt

        if action == 3:
            return ego_y, current_speed

        if action == 4:
            return ego_y, current_speed-acc*decision_dt

        # left lane action
        left_lane_index = ego_y+1
        if left_lane_index >= len(self._dynamic_map.mmap.lanes):
            if action == 5 or action == 6 or action == 7:
                return self._rule_based_lateral_model_instance.lateral_decision(self._dynamic_map)
        if action == 5:
            return left_lane_index, current_speed+acc*decision_dt

        if action == 6:
            return left_lane_index, current_speed

        if action == 7:
            return left_lane_index, current_speed-acc*decision_dt

        #right_lane_action
        right_lane_index = ego_y-1
        if right_lane_index < 0:
            if action == 8 or action == 9 or action == 10:
                return self._rule_based_lateral_model_instance.lateral_decision(self._dynamic_map)
        if action == 8:
            return right_lane_index, current_speed+acc*decision_dt

        if action == 9:
            return right_lane_index, current_speed

        if action == 10:
            return right_lane_index, current_speed-acc*decision_dt


    def get_state_from_lane(self,lane,lane_index,state,
                                range_x = 100,
                                range_vx = 50/3.6
                                ):
        if len(lane.front_vehicles) > 0:
            fv = lane.front_vehicles[0]
            fv_id = fv.uid
            fv_x = fv.ffstate.x
            fv_y = fv.ffstate.y
            fv_vx = fv.ffstate.vx
            # fv_vy = fv.mmap_vy # FIXME(zhong): should consider lane change speed
        else:
            # default fv # TODO(zhong): More reasonable default value
            fv_id = 0
            fv_x = range_x
            fv_y = lane_index
            fv_vx = range_vx
            # fv_vy = 0

        state.append(fv_x)
        state.append(fv_y)
        state.append(fv_vx)
        # state.append(fv_vy)

        if len(lane.rear_vehicles) > 0:
            rv = lane.rear_vehicles[0]
            rv_id = rv.uid
            rv_x = fv.ffstate.x # negative value
            rv_y = fv.ffstate.y
            rv_vx = fv.ffstate.vx
            # rv_vy = rv.mmap_vy # FIXME(zhong): should consider lane change speed
        else:
            rv_id = 0
            rv_x = -range_x
            rv_y = lane_index
            rv_vx = 0
            # rv_vy = 0
        state.append(rv_x)
        state.append(rv_y)
        state.append(rv_vx)
        # state.append(rv_vy)

        rospy.logdebug("lane %d: (%d)fv_x:%.1f, fv_y:%.1f, fv_vx:%.1f,||(%d)rv_x:%.1f, rv_y:%.1f, rv_vx:%.1f", lane_index,
                                        fv_id,fv_x,fv_y,fv_vx,rv_id,rv_x,rv_y,rv_vx)
