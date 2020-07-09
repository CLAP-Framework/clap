#!/usr/bin/env python
# Created by Minghan
import os

import numpy as np

import kitti_util

from generate_lidar_module import PclGenerator

import rospy
from zzz_common.params import parse_private_args

class PclGenerateNode(object):
    def __init__(self):
        self.args = parse_private_args(
            input_topic=["img_L", "img_R"],
            output_topic="objects_filtered", 
            calib_dir='~/Kitti/object/training/calib', 
            disparity_dir='~/Kitti/object/training/predicted_disparity', 
            save_dir='~/Kitti/object/training/predicted_velodyne',
            max_high=1
        )

        self.pcl_generator = PclGenerator(self.args)

    def run_from_file(self):
        assert os.path.isdir(self.args.disparity_dir)
        assert os.path.isdir(self.args.calib_dir)

        if not os.path.isdir(self.args.save_dir):
            os.makedirs(self.args.save_dir)
        
        disps = [x for x in os.listdir(self.args.disparity_dir) if x[-3:] == 'png' or x[-3:] == 'npy']
        disps = sorted(disps)

        for fn in disps:
            predix = fn[:-4]
            calib_file = '{}/{}.txt'.format(self.args.calib_dir, predix)
            calib = kitti_util.Calibration(calib_file)

            # disp_map = ssc.imread(args.disparity_dir + '/' + fn) / 256.
            disp_map = np.load(self.args.disparity_dir + '/' + predix+'.npy')
            disp_map = (disp_map*256).astype(np.uint16)/256.

            lidar = self.pcl_generator.run(calib, disp_map)

            # pad 1 in the indensity dimension
            lidar = np.concatenate([lidar, np.ones((lidar.shape[0], 1))], 1)
            lidar = lidar.astype(np.float32)
            lidar.tofile('{}/{}.bin'.format(self.args.save_dir, predix))
            print('Finish Depth {}'.format(predix))


def main():
    detection_node = DisparityNode()
    detection_node.run_from_file()

if __name__ == "__main__":
    main()