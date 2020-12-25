from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import socket
import sys
import time
import weakref

import matplotlib.pyplot as plt
import msgpack
import networkx as nx
import numpy as np

import gym
from gym import core, error, spaces, utils
from gym.utils import seeding

# from carla import Location, Rotation, Transform

##########################################

class ZZZCarlaEnv(gym.Env):
    metadata = {'render.modes': []}
    def __init__(self, zzz_client="127.0.0.1", port=2333, recv_buffer=4096, socket_time_out = 1000):
    
        self._restart_motivation = 0
        self.state = []
        self.steps = 1
        self.collision_times = 0

        # Socket
        socket.setdefaulttimeout(socket_time_out) # Set time out
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(socket_time_out) # Set time out
        self.sock.bind((zzz_client, port))
        self.sock.listen()
        self.sock_conn = None
        self.sock_buffer = recv_buffer
        self.sock_conn, addr = self.sock.accept()
        self.sock_conn.settimeout(socket_time_out) # Set time out
        self.rule_based_action = []
        print("ZZZ connected at {}".format(addr))

        # Set action space
        self.action_space = spaces.Discrete(9) # len(fplist +1) 
        # 0: rule-based policy

        self.state_dimention = 16

        low  = np.array([-100,  -100,   -20,  -20,  -100, -100,  -20,   -20,   -100, -100,   -20,  -20, -100,  -100, -20, -20])
        high = np.array([100, 100, 20, 20, 100, 100, 20, 20, 100, 100, 20, 20,100, 100, 20, 20])    

        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.seed()


    def step(self, action):

        # send action to zzz planning module
        
        action = action.astype(float)
        action = action.tolist()
        print("-------------",type(action),action)
        while True:
            try:
                send_action = action         
                self.sock_conn.sendall(msgpack.packb(send_action))

                # wait next state
                received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
                print("[GYM]: Received msg in step",received_msg)
                self.state = received_msg[0:16]
                collision = received_msg[16]
                leave_current_mmap = received_msg[17]
                

                # calculate reward
                reward = 0
                
                # judge if finish
                done = False

                if collision:
                    done = True
                    reward = -10#-1000
                    print("[CARLA]: Vehicle received collision")
                
                if leave_current_mmap == 1:
                    done = True
                    reward = 1#+500
                    print("[CARLA]: Successful pass intersection")

                elif leave_current_mmap == 2:
                    done = True
                    print("[CARLA]: Restart by code")
                
                # self.record_rl_intxt(action, q_value, RLpointx, RLpointy, rule_q, collision, leave_current_mmap, ego_s, threshold)
                return np.array(self.state), reward, done,  {}
            except:
                print("[GYM]: Not Received msg in step")
                reward = 0 
                done = False
                return np.array(self.state), reward, done,  {}
            

    def reset(self, **kargs):
       
        # receive state
        # if the received information meets requirements
        while True:
            try:
            
                received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
                print("[GYM]: Received msg in reset",received_msg)

                self.state = received_msg[0:16]
                collision = received_msg[16]
                leave_current_mmap = received_msg[17]
            

                return np.array(self.state)

                # if not collision and not leave_current_mmap:
            except:
                print("[GYM]: Not received msg in reset")
                collision = 0
                leave_current_mmap = 0
                return np.array(self.state)

        return np.array(self.state)


    def render(self, mode='human'):
        # if mode == 'human':
        #     screen_width = 600
        #     screen_height = 400
        #     #world_width = self.problem.xrange
        #     super(MyEnv, self).render(mode=mode)
        pass

