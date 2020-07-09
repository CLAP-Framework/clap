#!/usr/bin/env python
# Created by Minghan
import skimage
import skimage.io
import skimage.transform
import numpy as np
import time
import math

from pred_disp_module import DisparityNet

import rospy
from zzz_common.params import parse_private_args

# from munch import munchify

class DisparityNode(object):
    def __init__(self):
        self.args = parse_private_args(
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

        # network initialization
        self.disp_pred_net = DisparityNet(self.args)

    #     self._subscriber = rospy.Subscriber(self.args.pop("input_topic"), DetectionBoxArray, self.callback)
    #     self._publisher = rospy.Publisher(self.args.pop("output_topic"), DetectionBoxArray, queue_size=1)

    # def callback(self, msg):
    #     img_L_o = self.msg_to_img(msg)
    #     img_R_o = None
    #     disparity = self.disp_pred_net.run(img_L_o, img_R_o)
    #     out_message = img_to_msg(disparity)
    #     self._publisher.publish(out_message)

    def run_from_file(self):
        if self.args.KITTI == '2015':
            from dataloader import KITTI_submission_loader as DA
        else:
            from dataloader import KITTI_submission_loader2012 as DA  
        test_left_img, test_right_img = DA.dataloader(self.args.datapath)
    
        if not os.path.isdir(self.args.save_path):
            os.makedirs(self.args.save_path)

    
        for inx in range(len(test_left_img)):
            imgL_o = (skimage.io.imread(test_left_img[inx]).astype('float32'))
            imgR_o = (skimage.io.imread(test_right_img[inx]).astype('float32'))
    
            img = self.disp_pred_net.run(imgL_o, imgR_o)
            
            # file output
            print(test_left_img[inx].split('/')[-1])
            if self.args.save_figure:
                skimage.io.imsave(self.args.save_path+'/'+test_left_img[inx].split('/')[-1],(img*256).astype('uint16'))
            else:
                np.save(self.args.save_path+'/'+test_left_img[inx].split('/')[-1][:-4], img)


def main():
    detection_node = DisparityNode()
    detection_node.run_from_file()

if __name__ == "__main__":
    main()