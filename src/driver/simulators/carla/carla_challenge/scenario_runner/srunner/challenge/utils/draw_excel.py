#!/usr/bin/env python
# -*- coding: utf-8 -*-

import xlwt
import xlrd
from xlutils.copy import copy
from collections import Counter

from define_and_config import Common_Define, Common_Config
from draw_routes import DrawRoute
from draw_velocity import DrawVelocity
import threading

class DrawExcel(object):

    def write_excel_xls(self):
        workbook = xlwt.Workbook()  # 新建一个工作簿
        sheet = workbook.add_sheet('sheet1')  # 在工作簿中新建一个表格
        style = "font:colour_index red;"
        red_style = xlwt.easyxf(style)
        labels = ['num','route_name', 'route_length', 'left_turn', 'right_turn', 'collision_static', 'collision_vehicle','collision_pedestrian', \
        'red_light','wrong_way','route_deviation','sidewalk_invasion','stop_infraction','scenario_type']
        for i in range(0,len(labels)):
            sheet.write(0, i, labels[i],red_style)  # 
        path = Common_Define.first_path + 'result_static.xls'
        workbook.save(path)  # 保存工作簿
        print("xls格式表格创建成功！")

    def write_excel_xls_append(self, value):
        index = len(value)  # 获取需要写入数据的行数
        workbook = xlrd.open_workbook(Common_Define.first_path + 'result_static.xls')  # 打开工作簿
        sheets = workbook.sheet_names()  # 获取工作簿中的所有表格
        worksheet = workbook.sheet_by_name(sheets[0])  # 获取工作簿中所有表格中的的第一个表格
        rows_old = worksheet.nrows  # 获取表格中已存在的数据的行数
        new_workbook = copy(workbook)  # 将xlrd对象拷贝转化为xlwt对象
        new_worksheet = new_workbook.get_sheet(0)  # 获取转化后工作簿中的第一个表格
        for i in range(0, index):
            new_worksheet.write(0+rows_old, i, value[i])  # 追加写入数据，注意是从i+rows_old行开始写入
        # for i in range(0, index):
        #     for j in range(0, len(value[i])):
        #         new_worksheet.write(i+rows_old, j, value[i][j])  # 追加写入数据，注意是从i+rows_old行开始写入
        
        path = Common_Define.first_path + 'result_static.xls'
        new_workbook.save(path)  # 保存工作簿
        print("xls格式表格【追加】写入数据成功！")

    def write_excel_total(self):
        workbook = xlrd.open_workbook(Common_Define.first_path + 'result_static.xls')
        sheet_value = workbook.sheets()[0]
        sheets = workbook.sheet_names()  # 获取工作簿中的所有表格
        worksheet = workbook.sheet_by_name(sheets[0])  # 获取工作簿中所有表格中的的第一个表格
        rows_old = worksheet.nrows  # 获取表格中已存在的数据的行数
        # if rows_old < 3:
        #     return
        new_workbook = copy(workbook)  # 将xlrd对象拷贝转化为xlwt对象
        sheet1 = new_workbook.get_sheet(0)  # 获取转化后工作簿中的第一个表格
        # sheet1 = book.sheets()[0]
        
        nrows = worksheet.nrows
        if nrows < 2:
            return
        print('数据总行数 {}'.format(nrows-1))
        ncols = worksheet.ncols
        sheet1.write(nrows + 1, 0, 'total')

        temp_length = 0.0
        for row in range(1, nrows):
            temp_length += float((sheet_value.cell(row, 2).value))
        sheet1.write(nrows+1, 2, temp_length)

        
        for col in range(3 , ncols-1):
            temp = 0
            for row in range(1, nrows):
                temp += int((sheet_value.cell(row, col).value))
            sheet1.write(nrows+1, col, temp)
        
        scenario_list = []
        for row in range(1, nrows):
            scenario_list.append(sheet_value.cell(row, ncols-1).value.encode('gbk'))
        dict = {}
        for key in scenario_list:
            dict[key] = dict.get(key, 0) + 1
        sheet1.write(nrows+3, 0, 'scenario_type')
        sheet1.write(nrows+4, 0, 'scenario_num')
        temp_col = 2
        for i in  range(len(dict.keys())): 
            sheet1.write(nrows+3, temp_col, dict.keys()[i] )
            temp_col += 1
        temp_col1 = 2
        for i in  range(len(dict.values())):
            sheet1.write(nrows+4, temp_col1, dict.values()[i])
            temp_col1 += 1
        path = Common_Define.first_path + 'result_static.xls'
        new_workbook.save(path)  # 保存工作簿

        

class ResultStaticThread (threading.Thread):
    def __init__(self, value):
        threading.Thread.__init__(self)
        self.value = value

    def run(self):
        drawExcel = DrawExcel()
        drawExcel.write_excel_xls_append(self.value)
        # drawExcel.write_excel_total()
        Common_Define.excel_rows += 1
        Common_Define.left_turn_num , Common_Define.right_turn_num = 0, 0
        Common_Define.COLLISION_STATIC_NUM, Common_Define.COLLISION_VEHICLE_NUM, Common_Define.COLLISION_VEHICLE_NUM = 0, 0, 0
        Common_Define.TRAFFIC_LIGHT_NUM, Common_Define.WRONG_WAY_NUM, Common_Define.ROUTE_DEVIATION, Common_Define.SIDEWALK_INVASION_NUM, Common_Define.STOP_NUM = 0, 0 ,0 ,0 , 0
        # draw_route = DrawRoute()
        # draw_route.draw_routes(Common_Define.third_path + "vehicle_location_file.txt", Common_Define.third_path + "ref_waypoint_file.txt")
        # draw_velocity = DrawVelocity()
        # draw_velocity.draw_velocity(Common_Define.third_path + "velocity_file.txt")