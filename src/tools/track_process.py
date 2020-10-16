#!/usr/bin/python2
# -*- coding: utf-8 -*-
"""
Cubic Spline library on python

author Atsushi Sakai

usage: see test codes as below

license: MIT
"""
import os
import math

import numpy as np
import scipy as sp
from scipy import interpolate
import matplotlib.pyplot as plt
import numpy.linalg as LA
from scipy.interpolate import *

import bisect
import unittest

import rospy
import utm
import rosbag
import argparse

import tf
import sys
if sys.version > '3':
    from queue import Queue
else:
    from Queue import Queue

"""
origin definations 

tus-park : 442406,4427332
heqing : 
new shougang : 426660.51591584098,4417656.7450509444

"""


show_image = True

class Spline:
    """
    Cubic Spline class
    """
    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i+1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i+1] - self.a[i]) / h[i] - h[i] * (self.c[i+1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position
        if t is outside of the input x, return None
        """
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative
        if t is outside of the input x, return None
        """
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i+1, i+1] = 2.0 * (h[i] + h[i+1])
            A[i+1, i] = h[i]
            A[i, i+1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i+1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class
    """
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        '''
        curve length integral
        '''
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


def calc_curvature_with3points(x, y):
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    refer to https://github.com/Pjer-zhang/PJCurvature for detail
    """
    t_a = LA.norm([x[1] - x[0], y[1] - y[0]])
    t_b = LA.norm([x[2] - x[1], y[2] - y[1]])
    
    M = np.array([
        [1, -t_a, t_a**2],
        [1, 0,    0     ],
        [1,  t_b, t_b**2]
    ])
    a = np.matmul(LA.inv(M), x)
    b = np.matmul(LA.inv(M), y)
    kappa = 2 * (a[2]*b[1] - b[2]*a[1]) / (a[1]**2. + b[1]**2.) ** (1.5)
    
    return kappa, [b[1], -a[1]] / np.sqrt(a[1]**2. + b[1]**2.)


def process_bags(args):
    
    if os.path.exists(args.input) is False:
        raise  Exception('Input file not existed ! {}'.format(args.input))
    
    origins = args.origin.split(',')
    ox, oy = float(origins[0]), float(origins[1])
    
    input_bag = rosbag.Bag(args.input)
    
    input_list = []
    orientation = None
    last_point = None
    
    for topic, msg, _ in input_bag.read_messages(topics=[
        '/gps/fix', '/imu/data', '/localization/gps/fix', '/localization/imu/data']):

        if topic == '/gps/fix' or topic == '/localization/gps/fix':
            point = utm.from_latlon(msg.latitude, msg.longitude)
            if last_point is None:
                last_point = point

            vb = np.array([point[0], point[1]])
            va = np.array([last_point[0], last_point[1]])
            gap = np.linalg.norm(vb - va)
            if gap > 0.5 and orientation is not None:
                last_point = point
                input_list.append([point[0]-ox, point[1]-oy, 
                    orientation.x, orientation.y, orientation.z, orientation.w])
                
        if topic == '/imu/data' or topic == '/localization/imu/data':
            orientation = msg.orientation

    # print('+++++++++++++++++\n')
    # print(type(input_list))
    # print('+++++++++++++++++\n')
    
    input_data = np.array(input_list)
    x_array = input_data[:, 0]
    y_array = input_data[:, 1]

    spatial_x_array = x_array[1 : len(x_array) : 4]
    spatial_y_array = y_array[1 : len(y_array) : 4]
    
    x, y = spatial_x_array, spatial_y_array
    sp = Spline2D(x, y)
    # split as 0.2m
    s = np.arange(0, sp.s[-1], 0.2)
    
    # curvature
    points3_x, points3_y = Queue(3), Queue(3)
    
    output_data = []
    output2 = []

    # re-sample as 0.2m
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        
        yaw = sp.calc_yaw(i_s) 
        if yaw < 0.0 : # to [0, 2*PI)
            yaw += 2 * math.pi
            
        ryaw.append(yaw)
        pcurve = sp.calc_curvature(i_s)
        rk.append(pcurve)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw, 'sxyz')  
        # print('yaw = {},  q = {}\n'.format(yaw / math.pi * 180, q)) 
        
        # ignore div ZERO
        if pcurve == 0.0:
            r = 999999
        else:
            r = 1.0 / sp.calc_curvature(i_s)
            
        if points3_x.full() and points3_y.full():
            points3_x.get()
            points3_y.get()
        
        points3_x.put(ix)
        points3_y.put(iy)
        
        if points3_x.full() and points3_y.full():
            kappa, _ = calc_curvature_with3points(
                np.array(points3_x.queue), np.array(points3_y.queue))
            if kappa == 0.0:
                r = 999999
            else:
                r = 1.0 / kappa
        
        output_data.append([ix, iy, q[0], q[1], q[2], q[3], r])
        output2.append([ix, iy])
        
    if show_image:
        flg, ax = plt.subplots(1)
        plt.plot(spatial_x_array, spatial_y_array, "xb", label="input")
        plt.plot(rx, ry, "red", label="spline")
        plt.plot(x_array, y_array, color="blue", label="gps-track")
        plt.plot()
        
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.show()
        
    # print('+++++++++++++++++\n')
    # print(output_data)
    # print('+++++++++++++++++\n')   
        
    # save output_data
    np.savetxt(args.output, output_data, fmt="%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", delimiter=',', newline='\n')
    
    if args.o2 is not None:
        np.savetxt(args.o2, output2, fmt="%.8f,%.8f", delimiter=',', newline='\n')
    print('track process done!\n')
    


if __name__ == '__main__':
    # help : convert -origin 
    # shougang new origin 426660.51591584098, 4417656.7450509444
    # 428191,4417667
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--origin', required=True, help='like 426660.51591584098,4417656.7450509444  428191,4417667')
    arg_parser.add_argument('--input',  required=True, help='gps track bag file path')
    arg_parser.add_argument('--output', required=True, help='points(x,y,w) file path')
    arg_parser.add_argument('--o2', required=False, help='points(x,y) file path')

    args = arg_parser.parse_args()
    process_bags(args)
