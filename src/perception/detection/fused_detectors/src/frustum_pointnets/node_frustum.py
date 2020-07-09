#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge

from zzz_common.params import parse_private_args
from frustum-pointnets.module_wrapper import FrustumModule

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo, DetectionBox2DArray, DetectionBox2D, Pose2DWithCovariance
from sensor_msgs.msg import DetectionBoxArray, DetectionBox, BoundingBox, PoseWithCovariance

import message_filters

import pseudo_lidar.preprocessing.cam_velo_utils as cam_velo_utils

import argparse
import os
import numpy as np
import ros_numpy

def msg_detections_2d_to_bbox_and_labels(DetectionBox2DArray message_detection_2d):
    detections_2d = message_detection_2d.detections
    boxes = []
    labels = []
    for detection_2d in detections_2d:
        box_2d = detection_2d.bbox
        x_center = box_2d.pose.x
        y_center = box_2d.pose.y
        x_dim = box_2d.dimension.length_x
        y_dim = box_2d.dimension.length_y
        x_min = x_center - x_dim/2.
        x_max = x_center + x_dim/2.
        y_min = y_center - y_dim/2.
        y_max = y_center + y_dim/2.
        box = [x_min, y_min, x_max, y_max]
        boxes.append(box)

        cls_detect = detection_2d.classes[0].classid
        if np.bitwise_and(cls_detect, 1):
            cls_string = "Car"
        elif np.bitwise_and(cls_detect, 2):
            cls_string = "Pedestrian"
        elif np.bitwise_and(cls_detect, 3):
            cls_string = "Cyclist"

        labels.append(cls_string)

    return boxes, labels

def generate_3d_detection_msg(hs, ws, ls, txs, tys, tzs, rys, type_list):
    msg_detects_3d = DetectionBoxArray()
    for i in len(hs):
        detect_3d = DetectionBox()
        detect_3d.bbox.pose.position.x = txs[i]
        detect_3d.bbox.pose.position.y = tys[i]
        detect_3d.bbox.pose.position.z = tzs[i]
        detect_3d.bbox.pose.orientation.x = 0
        detect_3d.bbox.pose.orientation.y = 0
        detect_3d.bbox.pose.orientation.z = np.sin(rys[i]/2.)
        detect_3d.bbox.pose.orientation.w = np.cos(rys[i]/2.)
        detect_3d.bbox.dimension.length_x = ws[i]
        detect_3d.bbox.dimension.length_y = hs[i]
        detect_3d.bbox.dimension.length_z = ls[i]
        obj_class = ObjectClass()
        if type_list[i] == "Car":
            obj_class.classid = 1
        elif type_list[i] == "Pedestrian":
            obj_class.classid = 2
        elif type_list[i] == "Cyclist":
            obj_class.classid = 3
        obj_class.comments = type_list[i]
        detect_3d.classes.append(obj_class)

        msg_detects_3d.append(detect_3d)
    return msg_detects_3d

        
        
        
        
        

class Detection3dOnLidar(object):
    def __init__(self):
        params = parse_private_args(
            input_topic_left="/zzz/driver/image_raw",
            input_topic_lidar="/zzz/driver/image_raw",
            camera_info_left="/zzz/driver/camera_info",
            input_topic_detc_2d="", 
            output_topic="points_pseudo",
            target_frame="base_link" # TODO: which frame
        )

        # network initialization
        self.frustum_3d_detector = FrustumModule()

        self._bridge = CvBridge()
        
        self._publisher = rospy.Publisher(params.pop("output_topic"), DetectionBoxArray, queue_size=1)

        image_sub = message_filters.Subscriber(params.input_topic_left, Image)
        info_left_sub = message_filters.Subscriber(params.camera_info_left, CameraInfo)
        lidar_sub = message_filters.Subscriber(params.input_topic_lidar, PointCloud2)
        detection_2d_sub = message_filters.Subscriber(params.input_topic_detc_2d, DetectionBox2DArray)

        # img_cache = message_filters.Cache(image_sub, 30)
        # lidar_cache = message_filters.Cache(lidar_sub, 30)
        # info_left_sub = message_filters.Cache(info_left_sub, 30)

        ts = message_filters.TimeSynchronizer([image_sub, lidar_sub, detection_2d_sub, info_left_sub], 20) # TODO: the four message does not come at the same time
        ts.registerCallback(self.callback)

    def callback(self, msg_image_left, msg_lidar, message_detection_2d, info_cam_left):
        # process messages
        self._image_left = self._bridge.imgmsg_to_cv2(msg_image_left, "bgr8")
        imgL_o = self._image_left.astype('float32')

        self.K_left = info_cam_left.K
        calib = cam_velo_utils.Calibration(self.K_left)

        pcl_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg_lidar)
        pcs_xyz = ros_numpy.point_cloud2.get_xyz_points(pcl_array).transpose() # float32
        pcs_color = ros_numpy.point_cloud2.split_rgb_field(pcl_array).astype(np.float32)
        pcs_intensity = 0.3*pcs_color['r'] + 0.59 * pcs_color['g'] + 0.11 * pcs_color['b']
        pcs_in = np.stack((pcs_xyz, pcs_intensity), axis=1)

        boxes_2d, labels = msg_detections_2d_to_bbox_and_labels(message_detection_2d)
        
        # run 3d detection given inputs
        hs, ws, ls, txs, tys, tzs, rys, type_list = \
            self.frustum_3d_detector.run(pcs_in, imgL_o, labels, boxes_2d, calib) # pcs_in is n*4 np array

        msg_detections_3d = generate_3d_detection_msg(hs, ws, ls, txs, tys, tzs, rys, type_list)
        msg_detections_3d.header = msg_image_left.header

        self._publisher.publish(msg_detections_3d)

        