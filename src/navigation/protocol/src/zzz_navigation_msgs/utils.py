'''
This package contains useful message conversions
'''

import numpy as np

def get_lane_array(lanes):
    arrays = []
    for lane in lanes:
        point_list = [(point.position.x, pose.position.y) for point in lane.central_path_points]
        arrays.append(np.array(point_list))
    return arrays
