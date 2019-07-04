import argparse
import time
from sys import platform
import os.path as osp

from .models import *
from .yutils.datasets import *
from .yutils.utils import *

from skimage import io

import socket
import msgpack

from .write_ipm_yml import CamIntrExtr

yolo_base_path = os.path.abspath(os.path.dirname(__file__))
default_config = osp.join(yolo_base_path, "cfg", "yolov3-spp.cfg")
default_data = osp.join(yolo_base_path, "data", "coco.data")
default_weights = osp.join(yolo_base_path, "weights", "yolov3-spp.weights")

class YoloDetect():
    def __init__(self, cfg=default_config,
        data_cfg=default_data,
        weights=default_weights,
        # images='data/samples',
        output='output',  # output folder
        img_size=416,
        conf_thres=0.5,
        nms_thres=0.5,
        save_txt=False,
        save_images=False,
        webcam=False):
        
        self.device = torch_utils.select_device()
        if os.path.exists(output):
            shutil.rmtree(output)  # delete output folder
        os.makedirs(output)  # make new output folder
        self.output = output
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.save_txt = save_txt
        self.save_images = save_images
        self.webcam = webcam

        self.img_size = img_size
        # Initialize model
        self.model = Darknet(cfg, self.img_size)

        # Load weights
        if weights.endswith('.pt'):  # pytorch format
            self.model.load_state_dict(torch.load(weights, map_location=self.device)['model'])
        else:  # darknet format
            _ = load_darknet_weights(self.model, weights)

        self.model.to(self.device).eval()

        # Get classes and colors
        self.classes = load_classes(parse_data_cfg(data_cfg)['names'])
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.classes))]

        # initialize camera geometry model
        self.cam = CamIntrExtr(0, 10, 0, 2.2)
        self.cam.setIntr(300, 150, fov=70)
        

    def run(self, im0, frame_id=0):
        if im0 is None or im0 is False or im0.shape != (300, 400, 3): # im0.shape != (480, 640, 3)
            print("skipping")
            time.sleep(.01)
            # continue

        # print(im0.shape)
        # print(im0.shape == (480, 640, 3))
        # im0 = im0[:, :, ::-1].astype(np.uint8).copy() 
        img, _, _, _ = letterbox(im0, height = self.img_size)

        # Normalize RGB
        img = img.transpose(2, 0, 1)  # RGB, channel first
        # img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, channel first
        img = np.ascontiguousarray(img, dtype=np.float32)  # uint8 to float32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        # print(img.shape)
        # cv2.imshow("original image", img0)
        # cv2.waitKey(0)

        path = 'frame%d.jpg'%(frame_id)

        t = time.time()
        save_path = str(Path(self.output) / Path(path).name)
        # print("save path:" , os.path.abspath(save_path))

        # Get detections
        img = torch.from_numpy(img).unsqueeze(0).to(self.device)
        if ONNX_EXPORT:
            torch.onnx.export(self.model, img, 'weights/model.onnx', verbose=True)
            return
        pred, _ = self.model(img)
        detections = non_max_suppression(pred, self.conf_thres, self.nms_thres)[0]

        print('Frame %d:' % (frame_id) )
        # print('Frame %d:' % (frame_id), end=' ')

        # send_info = []
        pos_list = []
        if detections is not None and len(detections) > 0:
            # Rescale boxes from 416 to true image size
            scale_coords(self.img_size, detections[:, :4], im0.shape).round()

            # Print results to screen
            for c in detections[:, -1].unique():
                n = (detections[:, -1] == c).sum()
                # print('%g %ss' % (n, self.classes[int(c)]), end=', ')
                print('%g %ss' % (n, self.classes[int(c)]) )

            # Draw bounding boxes and labels of detections
            # pos_list = [None]*detections.shape[0]
            # for *xyxy, conf, cls_conf, cls in detections:
            for x1, y1, x2, y2, conf, cls_conf, cls in detections:
                xyxy = (x1, y1, x2, y2)
                if self.save_txt:  # Write to file
                    with open(save_path + '.txt', 'a') as file:
                        file.write(('%g ' * 6 + '\n') % (x1, y1, x2, y2, cls, conf))

                # Add bbox to the image
                label = '%s %.2f' % (self.classes[int(cls)], conf)
                # pos = plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)]) # None or [lat, long]
                pos = self.plot_one_box_on_image(xyxy, im0, label=label, color=self.colors[int(cls)], labelclass=self.classes[int(cls)])
                # pos = self.calc_real_world_pos(xyxy, labelclass=self.classes[int(cls)])
                pos_list.append(pos)
                
        # print('(%.3fs)' % (time.time() - t))
        print(' ')
        cv2.imshow('yolo output', im0[:, :,::-1]) # 
        cv2.waitKey(1)
        # imgplot = plt.imshow(im0)
        # # imgplot = plt.imshow(im0[:,:,::-1])
        # plt.pause(0.01)
        
        if self.save_images:  # Save generated image with detections
            cv2.imwrite(save_path, im0)

        return pos_list

    def plot_one_box_on_image(self, x, img, color=None, label=None, line_thickness=None, labelclass=None):
        # Plots one bounding box on image img
        tl = line_thickness or round(0.002 * max(img.shape[0:2])) + 1  # line thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=int(tl) )
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=int(tf) )[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=int(tf), lineType=cv2.LINE_AA)

            ### calculate real world position relative to self vehicle 
            # pos = cal_real_pos(int(x[0] + x[2])/2, int(x[3]), img.shape)

            # u = int(x[0] + x[2])/2
            # v = int(x[3])
            # u = np.array([u])
            # v = np.array([v])
            # lo, la = self.cam.img2world(u, v)
            # pos = (lo[0], la[0], labelclass)
            pos = self.calc_real_world_pos(x, labelclass)
            lo = pos[0]
            la = pos[1]
            
            # cv2.putText(img, 'lo %.1f la %.1f'%(pos[1], pos[0]), ((x[0] + x[2])/2, x[3]+2), 0, tl/3, [255, 0, 0], thickness=tf, lineType=cv2.LINE_AA)
            cv2.putText(img, 'lo %.1f'%(lo), ((x[0] + x[2])/2, x[3]+5), 0, tl/3 - 0.2, [255, 0, 0], thickness=int(tf), lineType=cv2.LINE_AA)
            cv2.putText(img, 'la %.1f'%(la), ((x[0] + x[2])/2, x[3]+20), 0, tl/3 - 0.2, [255, 0, 0], thickness=int(tf), lineType=cv2.LINE_AA)
            return pos
        else:
            return None

    def calc_real_world_pos(self, x, labelclass):
        u = int(x[0] + x[2])/2
        v = int(x[3])
        u = np.array([u])
        v = np.array([v])
        lo, la = self.cam.img2world(u, v)  #lo front, la right
        pos = (lo[0], la[0], labelclass)
        return pos

