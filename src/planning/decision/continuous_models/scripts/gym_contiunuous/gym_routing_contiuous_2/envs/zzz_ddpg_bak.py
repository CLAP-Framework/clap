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

global s 
s = 19
global ss
ss = 10

class ZZZCarlaEnv(gym.Env):
    metadata = {'render.modes': []}
    def __init__(self, zzz_client="127.0.0.1", port=2333, recv_buffer=4096, socket_time_out = 100000):
    
        self._restart_motivation = 0
        self.state = []
        self.steps = 1
        self.collision_times = 0

        # Socket
        # socket.setdefaulttimeout(socket_time_out) # Set time out
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.sock.settimeout(socket_time_out) # Set time out
        # self.sock.bind((zzz_client, port))
        # self.sock.listen()
        # self.sock_conn = None
        # self.sock_buffer = recv_buffer
        # self.sock_conn, addr = self.sock.accept()
        # self.sock_conn.settimeout(socket_time_out) # Set time out
        # self.rule_based_action = []
        # print("ZZZ connected at {}".format(addr))

        # Set action space
        self.action_space = spaces.Discrete(17) # len(fplist +1) 
        # 0: rule-based policy

        self.state_dimention = 16

        low  = np.array([10,  -2, 0, -2,-100,  -100,   -20,  -20,  10, -100,  -20,   -20,   -100, -100,   -20,  -20])#, -100,  -100, -20, -20])
        high = np.array([40, 2, 10, 2, 100, 100, 20, 20, 50, 100, 20, 20, 100, 100, 20, 20])#,100, 100, 20, 20])    

        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.seed()


    def step(self, action):
        global s
        global ss
        # send action to zzz planning module
        
        action = action.astype(float)
        action = action.tolist()
        print("-------------",type(action),action)
        while True:
            try:
                send_action = action         
                # self.sock_conn.sendall(msgpack.packb(send_action))

                # wait next state
                # received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
                # print("[GYM]: Received msg in step",received_msg)
                # self.state = received_msg[0:16]
                # collision = received_msg[16]
                # leave_current_mmap = received_msg[17]

                # temp
                s = s + 1 + random.random() * 0.2
                ss = ss + 1
                self.state = [s,random.random() * 1 - 0.5,4 + random.random() * 0.2,random.random() * 0.5 - 0.25,0,0,0,0,ss,-1,3,6,0,0,0,0]
                if s < 28 and s > 27 and ss < 30 and ss > 25:
                    collision = True
                    s = 19
                    ss = 10 + random.random() * 20
                else:
                    collision = False
                if s > 35:
                    leave_current_mmap = 1
                    s = 19
                    ss = 10 + random.random() * 20
                else:
                    leave_current_mmap = 0
                print(self.state)

                # step reward
                reward = 0#self.state[2] * 0.01
                
                # judge if finish
                done = False

                if collision:
                    done = True
                    reward = -10
                    print("[CARLA]: Vehicle received collision")
                
                if leave_current_mmap == 1 and collision == False:
                    done = True
                    reward = 10
                    print("[CARLA]: Successful pass intersection")

                elif leave_current_mmap == 2 and collision == False:
                    reward = +1
                    done = True
                    print("[CARLA]: Restart by code")
                
                return np.array(self.state), reward, done,  {}
            except:
                print("[GYM]: Not Received msg in step")
                reward = 0 
                done = False
                return None
            

    def reset(self, **kargs):
       
        # receive state
        # if the received information meets requirements
        while True:
            try:
                global ss
                # received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
                # print("[GYM]: Received msg in reset",received_msg)

                # self.state = received_msg[0:16]
                # collision = received_msg[16]
                # leave_current_mmap = received_msg[17]
                # temp
                self.state = [s,random.random() * 1 - 0.5,4 + random.random() * 0.2,random.random() * 0.5 - 0.25,0,0,0,0,ss,-1,3,6,0,0,0,0]
                collision = 0
                leave_current_mmap = 0
                print(self.state)


                return np.array(self.state)

                # if not collision and not leave_current_mmap:
            except:
                print("[GYM]: Not received msg in reset")
                collision = 0
                leave_current_mmap = 0
                return None



    def render(self, mode='human'):
        # if mode == 'human':
        #     screen_width = 600
        #     screen_height = 400
        #     #world_width = self.problem.xrange
        #     super(MyEnv, self).render(mode=mode)
        pass

