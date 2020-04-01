from __future__ import print_function

import socket
import msgpack

import sys
import math
import numpy as np
import networkx as nx
import gym
import matplotlib.pyplot as plt
import argparse
import logging
import random
import time
import collections
import datetime
import glob
import os
import re
import weakref
import matplotlib.pyplot as plt


from gym import error, spaces, utils
from gym.utils import seeding

##########################################

class ZZZCarlaEnv_lane(gym.Env):
    metadata = {'render.modes': []}
    def __init__(self, zzz_client="127.0.0.1", port=2345, recv_buffer=4096):
        print("initilize")

        self.action_space = spaces.Discrete(8)
        """
        action space:
        0: rule-based policy
        1: emergency brake (acc = -10)
        2: acc = 0; target to outside
        3: acc = 0; target to inside
        4: acc = 1; target to outside
        5: acc = 1; target to inside
        6: acc = -1; target to outside
        7: acc = -1; target to inside
        """
        self._restart_motivation = 0
        self.state = None
        self.steps = 1
        self.collision_times = 0
        self.long_time_collision_flag = False
        self.long_time_collision_times = 0
        self.kick_in_times = 0
        # self._restart_motivation = 0
        # self.state = []
        # self.steps = 1
        # self.collision_times = 0
        self.state_dimention = 20
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((zzz_client, port))
        self.sock.listen()
        self.sock_conn = None
        self.sock_buffer = recv_buffer
 
        # x y vx vy
        low  = np.array([-1, -10,  0, -10,  0, -10,  0, -10,  0,-10,  0, -10, -50,-10,  0,-10,-50,-10,  0,-10])
        high = np.array([ 1,  10, 20,  10, 50,  10, 20,  10, 50, 10, 20,  10,   0, 10, 20, 10,  0, 10, 20, 10])

        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.seed()
        self._use_socked = True
        if self._use_socked:
            self.sock_conn, addr = self.sock.accept()
            print("ZZZ connected at {}".format(addr))
        else:
            self.test_rec_msg = [0, 0, 0, 0, 50, 0, 20, 0, 50, 1, 20, 0, -50, 0, 0, 0, -34.907974, 0.966466, 0, 0, 0, 0]
            print("Testing Mode")

    def step(self, action):
        action = action.astype(int)
        action = action.tolist()
        # send action to zzz planning module
        while True:
            try:
                if self._use_socked:
                    self.sock_conn.sendall(msgpack.packb(action))
                    received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
                else:
                    received_msg = self.test_rec_msg
                # wait next state
                self.state = received_msg[0:20]
                collision = received_msg[20]
                leave_current_map = received_msg[21]

                # calculate reward
                reward = 0
              
                # judge if finish
                done = False

                collision_happen = False
                if collision:
                    self.long_time_collision_flag = True
                    self.collision_times += 1
                    print("total_collision:",self.collision_times)
                    collision_happen = True
                    reward = -1
                    done = True

                steps = self.steps
                self.steps = steps + 1

                if leave_current_map:
                    done = True
                    reward = 0
                
                return np.array(self.state), reward, done, {"is_success": not collision_happen}

            except:
                print("RL cannot receive an state")

                continue

        return np.array(self.state), reward, done, collision_happen


    def reset(self, **kargs):
        # receive state
        # if the received information meets requirements
        while True:
            try:
                action = 0
                if self._use_socked:
                    self.sock_conn.sendall(msgpack.packb(action))
                    received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
                else:
                    received_msg = self.test_rec_msg
                self.state = received_msg[0:20]
                collision = received_msg[20]
                leave_current_map = received_msg[21]

                if collision or leave_current_map:
                    continue

                return np.array(self.state)

            except ValueError:
                continue

        self.steps = 1   
        return np.array(self.state)

    def render(self, mode='human'):
        if mode == 'human':
            screen_width = 600
            screen_height = 400
            #world_width = self.problem.xrange
            super(MyEnv, self).render(mode=mode)
