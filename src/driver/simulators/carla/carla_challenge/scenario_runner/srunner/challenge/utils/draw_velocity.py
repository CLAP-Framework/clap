#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import argparse

class DrawVelocity(object):

    def draw_velocity(self, velocity_file_path):
        x0 = []
        y0 = []
        y_temp = 0.0
        temp_max = 0.0
        temp_min = 0.0

        velocity_file = open(velocity_file_path)  
        lines = velocity_file.readlines() 
        for li in lines: 
            y0.append(float(li))
            if temp_max < float(li):
                temp_max = float(li)
            x0.append(y_temp)
            y_temp += 1

        font1 = {'family' : 'Times New Roman',  
        'weight' : 'normal',  
        'size'   : 9,  
        } 
        plt.figure(figsize=(22,6))    # 此行代码如果不是plt第一行，会生成两个图片
        plt.title('Velocity Result Analysis')
        plt.ylabel("Velocity Value (m/s) ")

        plt.plot(x0, y0, color='black', label='$Velocity$', linewidth=0.8)  # 绘制，指定颜色、标签、线宽，标签采用latex格式
        plt.ylim(-1, int(temp_max + 2))                                           # 设定y轴范围
        plt.xlim(-1, int(y_temp))                                             # 设定x轴范围
        plt.legend()

        plt.savefig(velocity_file_path + "velocity.png", dpi=600)
        # plt.show()
