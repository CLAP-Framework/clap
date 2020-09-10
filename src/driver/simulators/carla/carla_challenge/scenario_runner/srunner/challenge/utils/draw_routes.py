#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt1
import numpy as np
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import argparse

class DrawRoute(object):

    def draw_routes(self,vehicle_location_file_path, ref_waypoint_file_path):
        x0 = []
        y0 = []
        x1 = []
        y1 = []
        
        vehicle_location_file = open(vehicle_location_file_path)  
        lines = vehicle_location_file.readlines() 
        x0_min = 10000000000000000
        x0_max = -10000000000000000
        y0_min = 10000000000000000
        y0_max = -10000000000000000
        for li in lines: 
            x0.append(float(li.split( )[0]))
            y0.append(float(li.split( )[1]))
            if x0_min > float(li.split( )[0]):
                x0_min = float(li.split( )[0])
            if x0_max < float(li.split( )[0]):
                x0_max = float(li.split( )[0])
            if y0_min > float(li.split( )[1]):
                y0_min = float(li.split( )[1])
            if y0_max < float(li.split( )[1]):
                y0_max = float(li.split( )[1])
    

        x1_min = 10000000000000000
        x1_max = -10000000000000000
        y1_min = 10000000000000000
        y1_max = -10000000000000000
        t_max = 0
        t_min = 0
        ref_waypoint_file = open(ref_waypoint_file_path)
        waypoints_lines = ref_waypoint_file.readlines()
        for w_li in waypoints_lines:
            x1.append(float(w_li.split( )[0]))
            y1.append(float(w_li.split( )[1]))
            if x1_min > float(w_li.split( )[0]):
                x1_min = float(w_li.split( )[0])
            if x1_max < float(w_li.split( )[0]):
                x1_max = float(w_li.split( )[0])
            if y1_min > float(w_li.split( )[1]):
                y1_min = float(w_li.split( )[1])
            if y1_max < float(w_li.split( )[1]):
                y1_max = float(w_li.split( )[1])

        font1 = {'family' : 'Times New Roman',  
        'weight' : 'normal',  
        'size'   : 9,  
        } 

        font2 = {'family' : 'Times New Roman',  
        'weight' : 'normal',  
        'size'   : 14,  
        }  

        # figsize = 13, 13
        # plt.subplots(figsize=figsize)                                # 设定整张图片大小
        plt1.figure(figsize=(20,20)) 
        plt1.title('Result Analysis')

        # ax1 = plt.subplot(4, 10, 1)
        # ax1.yaxis.set_major_locator(MultipleLocator(15))             # 设定y轴刻度间距

        plt1.plot(x0, y0, color='black', label='$vehicle location$', linewidth=0.8)  # 绘制，指定颜色、标签、线宽，标签采用latex格式
        y_min = min(y0_min, y1_min)
        y_max = max(y0_max, y1_max)
        # plt1.ylim(y_min + y_min/5, y_max + abs(y_max/5))                                           # 设定y轴范围
        plt1.xlabel("Horizontal coordinates (m)")
        plt1.ylabel("Vertical  coordinates (m) ")

        # hl=plt.legend(loc='upper right', prop=font1, frameon=False)   # 绘制图例，指定图例位置
        #第二条曲线
        plt1.plot(x1, y1, color='red', label='$ref waypoint$', linewidth=0.8)
        plt1.legend(loc='upper right', prop=font1, frameon=False)                                # 绘制图例，指定图例位置
        # ax1.xaxis.set_major_locator(MultipleLocator(15))  
        # plt.xticks([])                                               # 去掉x坐标轴刻度
        x_min = min(x0_min, x1_min) 
        x_max = max(x0_max, x1_max) 
        if x_max > y_max:
            t_max = x_max
        else:
            t_max = y_max
        if x_min < y_min:
            t_min = x_min
        else:
            t_min = y_min
        plt1.xlim(t_min - abs(t_min/5), t_max + abs(t_max/5))                                             # 设定x轴范围
        plt1.ylim(t_min - abs(t_min/5), t_max + abs(t_max/5))                                           # 设定y轴范围
        plt1.legend()
        plt1.savefig(vehicle_location_file_path + "compare_routes.png", dpi=600)
        # plt.show()
