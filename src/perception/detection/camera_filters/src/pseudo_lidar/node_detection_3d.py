#!/usr/bin/env python
# Created by Minghan
import skimage
import skimage.io
import skimage.transform
import numpy as np
import time
import math
import kitti_util

from psmnet.pred_disp_module import DisparityNet
from preprocessing.generate_lidar_module import PclGenerator

import rospy
from zzz_common.params import parse_private_args

class detection3dPseudeLidarNode(object):
    def __init__(self):
        self.args_disp = parse_private_args(
            input_topic=["img_L", "img_R"],
            output_topic="objects_filtered", 
            KITTI="2015", 
            datapath='/scratch/datasets/kitti2015/testing/', 
            loadmodel=None,
            model='stackhourglass', 
            maxdisp=192,
            no_cuda=False,
            seed=1,
            save_path='finetune_1000',
            save_figure=False
        )

        self.args_gen_lidar = parse_private_args(
            input_topic=["img_L", "img_R"],
            output_topic="objects_filtered", 
            calib_dir='~/Kitti/object/training/calib', 
            disparity_dir='~/Kitti/object/training/predicted_disparity', 
            save_dir='~/Kitti/object/training/predicted_velodyne',
            max_high=1
        )

        # network initialization
        self.disp_pred_net = DisparityNet(self.args_disp)
        self.pcl_generator = PclGenerator(self.args_gen_lidar)
        
    def run_from_file(self):
        if self.args_disp.KITTI == '2015':
            from dataloader import KITTI_submission_loader as DA
        else:
            from dataloader import KITTI_submission_loader2012 as DA  
        test_left_img, test_right_img = DA.dataloader(self.args_disp.datapath)
    
        if not os.path.isdir(self.args_disp.save_path):
            os.makedirs(self.args_disp.save_path)

    
        for inx in range(len(test_left_img)):
            imgL_o = (skimage.io.imread(test_left_img[inx]).astype('float32'))
            imgR_o = (skimage.io.imread(test_right_img[inx]).astype('float32'))
    
            img = self.disp_pred_net.run(imgL_o, imgR_o)

            # # file output
            # print(test_left_img[inx].split('/')[-1])
            # if self.args.save_figure:
            #     skimage.io.imsave(self.args.save_path+'/'+test_left_img[inx].split('/')[-1],(img*256).astype('uint16'))
            # else:
            #     np.save(self.args.save_path+'/'+test_left_img[inx].split('/')[-1][:-4], img)

            predix = test_left_img[inx].split('/')[-1][:-4]
            calib_file = '{}/{}.txt'.format(self.args_gen_lidar.calib_dir, predix)
            calib = kitti_util.Calibration(calib_file)

            img = (img*256).astype(np.uint16)/256.
            lidar = self.pcl_generator.run(calib, img)

            # pad 1 in the indensity dimension
            lidar = np.concatenate([lidar, np.ones((lidar.shape[0], 1))], 1)
            lidar = lidar.astype(np.float32)
            lidar.tofile('{}/{}.bin'.format(self.args_gen_lidar.save_dir, predix))
            print('Finish Depth {}'.format(predix))
            