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

##########################################

class ZZZCarlaEnv(gym.Env):
    metadata = {'render.modes': []}
    def __init__(self, zzz_client="127.0.0.1", port=2333, recv_buffer=4096):

       
        self._restart_motivation = 0
        self.state = []
        self.steps = 1
        self.collision_times = 0

        # Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((zzz_client, port))
        self.sock.listen()
        self.sock_conn = None
        self.sock_buffer = recv_buffer
        self.sock_conn, addr = self.sock.accept()
        print("ZZZ connected at {}".format(addr))


        # Set action space
        low_action = np.array([-50,-50])
        high_action = np.array([50,50])  #Should be symmetry for DDPG
        self.action_space = spaces.Box(low=low_action, high=high_action, dtype=np.float32)

        # Set State space = 4+4*obs_num

        # ego state: ego_x(0), ego_y(1), ego_vx(2), ego_vy(3)    
            # obstacle 0 : x0(4), y0(5), vx0(6), vy0(7)
            # obstacle 1 : x0(8), y0(9), vx0(10), vy0(11)
            # obstacle 2 : x0(12), y0(13), vx0(14), vy0(15)

        self.state_dimention = 16

        low  = np.array([-50,  -50,   0, 0,  0, -100, 0,  0,   0, 0,  0, -100, 0,  0,0,0])
        high = np.array([1, 17, 100, 1, 12,    0, 1, 12, 100, 1, 12,    0, 1, 12,0,0])    

        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.seed()


    def step(self, action):

        # send action to zzz planning module
        # self.sock_conn.sendall(msgpack.packb(int(action)))
        
        # wait next state
        # received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
        self.state = np.array([0,  0,   0, 0,  0, -100, 0,  0,   0, 0,  0, -100, 0,  0,0,0])
        # collision = received_msg[14]
        # leave_current_mmap = received_msg[15]
    
        # calculate reward
        reward = 1

        # if collision:
        #     reward = 0

        # judge if finish
        done = False

        # if collision:
        #     done = True
        
        # if leave_current_mmap:
        #     done = True

        return np.array(self.state), reward, done, {}


    def reset(self, **kargs):
       
        # receive state
        # if the received information meets requirements
        # while True:
        #     try:
        #         # received_msg = msgpack.unpackb(self.sock_conn.recv(self.sock_buffer))
        #         self.state = np.array([0,  0,   0, 0,  0, -100, 0,  0,   0, 0,  0, -100, 0,  0])
        #         # collision = received_msg[14]
        #         # leave_current_mmap = received_msg[15]
        #         if not collision and not leave_current_mmap:
        #             break
        #     except ValueError:
        #         continue
            
        return np.array([0,  0,   0, 0,  0, -100, 0,  0,   0, 0,  0, -100, 0,  0,0,0])#np.array(self.state)

    def render(self, mode='human'):
        if mode == 'human':
            screen_width = 600
            screen_height = 400
            #world_width = self.problem.xrange
            super(MyEnv, self).render(mode=mode)
