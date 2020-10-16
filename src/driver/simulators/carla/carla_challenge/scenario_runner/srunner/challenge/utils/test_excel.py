#!/usr/bin/env python
# -*- coding: utf-8 -*-

import xlwt
import xlrd
from xlutils.copy import copy
from collections import Counter
from define_and_config import Common_Define

workbook = xlrd.open_workbook('/home/zhangxinhong/shared_dir/carla-autoware/catkin_ws/z_record_file/20200421:102945/' + 'result_static.xls')
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
print('表格总行数 {}'.format(nrows))
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
for i in range(len(dict.values())):
    sheet1.write(nrows+4, temp_col1, dict.values()[i])
    temp_col1 += 1
path = 'result_static.xls'
new_workbook.save(path)  # 保存工作簿
        

        






