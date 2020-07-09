# created by Minghan
from __future__ import print_function
import argparse
import os
import random
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data
from torch.autograd import Variable
import torch.nn.functional as F
import skimage
import skimage.io
import skimage.transform
import numpy as np
import time
import math
from utils import preprocess 
from models import *


# 2012 data /media/jiaren/ImageNet/data_scene_flow_2012/testing/

class DisparityNet():
    def __init__(self, params):
        self.args = params
        self.args.cuda = not self.args.no_cuda and torch.cuda.is_available()

        torch.manual_seed(self.args.seed)
        if self.args.cuda:
            torch.cuda.manual_seed(self.args.seed)
        
        # load model
        if self.args.model == 'stackhourglass':
            self.model = stackhourglass(self.args.maxdisp)
        elif self.args.model == 'basic':
            self.model = basic(self.args.maxdisp)
        else:
            print('no model')

        self.model = nn.DataParallel(self.model, device_ids=[0])
        self.model.cuda()

        if self.args.loadmodel is not None:
            state_dict = torch.load(self.args.loadmodel)
            self.model.load_state_dict(state_dict['state_dict'])

        print('Number of model parameters: {}'.format(sum([p.data.nelement() for p in self.model.parameters()])))

        # process operations
        self.processed = preprocess.get_transform(augment=False)


    def test(self, imgL,imgR):
        self.model.eval()

        if self.args.cuda:
            imgL = torch.FloatTensor(imgL).cuda()
            imgR = torch.FloatTensor(imgR).cuda()     

        imgL, imgR= Variable(imgL), Variable(imgR)

        with torch.no_grad():
            output = self.model(imgL,imgR)
        output = torch.squeeze(output)
        pred_disp = output.data.cpu().numpy()

        return pred_disp

    def test_preprocess(self, imgL_o):
        
        imgL = self.processed(imgL_o).numpy()
        imgL = np.reshape(imgL,[1,3,imgL.shape[1],imgL.shape[2]])

        # pad to (384, 1248)
        top_pad = 384-imgL.shape[2]
        left_pad = 1248-imgL.shape[3]
        imgL = np.lib.pad(imgL,((0,0),(0,0),(top_pad,0),(0,left_pad)),mode='constant',constant_values=0)
        return imgL

    def run(self, imgL_o, imgR_o):
        imgL = self.test_preprocess(imgL_o)
        imgR = self.test_preprocess(imgR_o)

        # inference
        start_time = time.time()
        pred_disp = self.test(imgL,imgR)
        print('time for disparity prediction = %.2f' %(time.time() - start_time))

        # postprocess
        top_pad   = 384 - imgL_o.shape[0]
        left_pad  = 1248 - imgL_o.shape[1]
        if left_pad != 0:
            img = pred_disp[top_pad:,:-left_pad]
            return img
        return pred_disp

