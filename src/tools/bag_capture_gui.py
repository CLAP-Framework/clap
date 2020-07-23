#!/usr/bin/env python
# coding: utf-8
import os
import sys
import time
import threading
import copy
import signal

import roslib
import rospy
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridgeError, CvBridge
from queue import Queue
import argparse
import utm
import numpy as np

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rviz

# zzz message reference
from xpmotors_can_msgs.msg import AutoStateEx
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_perception_msgs.msg import TrackingBox, TrackingBoxArray, DetectionBox, DetectionBoxArray, ObjectSignals


roslib.load_manifest('rosbag')

# parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
# parser.add_argument('bagfile', nargs=1, help='input bag file')
# parser.add_argument('--window_seconds', nargs=1, help='max time offset (sec) to correct.', default='120', type=int)
# args = parser.parse_args()

# topic sub, pub
auto_topic      = '/xp/auto_state_ex'
ego_pose_topic  = '/zzz/navigation/ego_pose'
obs_topic       = '/zzz/perception/objects_tracked'
left_cam_topic  = '/left_usb_cam/image_raw/compressed'
pcl_topic       = '/middle/rslidar_points'

traffic_publisher   = None
traffic_light_topic = "/zzz/perception/traffic_lights"

# 2-autopilot, 0-manually
last_autostate = 0
take_over_count = 0
# bag record for 2 minutes
window_seconds = 60 * 2

last_point = None
total_distance = 0.0

# autostate_mode
auto_queue  = Queue(50 * window_seconds)
# ego_pose hz 100
pose_queue  = Queue(100 * window_seconds)
# perception obstacle hz 10
obs_queue   = Queue(10 * window_seconds)
# usb cam hz 10
image_queue = Queue(10 * window_seconds) 
# pcl hz 10
pcl_queue   = Queue(10 * window_seconds)


def start_capture(auto_queue, pose_queue, obs_queue, image_queue, pcl_queue):
    t = threading.Thread(
        target = write2bag, 
        args = (auto_queue, pose_queue, obs_queue, image_queue, pcl_queue))
    t.setDaemon(False)
    t.start()
    rospy.loginfo("*** launch thread record bag file ###")
    

def write2bag(auto_queue, pose_queue, obs_queue, image_queue, pcl_queue):
    filename = time.strftime("%Y-%m-%d_%H-%M-%S.bag", time.localtime()) 
    # time.sleep(60)
    bag = rosbag.Bag(filename, "w")
    for msg, t in list(auto_queue.queue):
        bag.write(auto_topic, msg, t)
    
    for msg, t in list(pose_queue.queue):
        bag.write(ego_pose_topic, msg, t)

    for msg, t in list(obs_queue.queue):
        bag.write(obs_topic, msg, t)

    for msg, t in list(image_queue.queue):
        bag.write(left_cam_topic, msg, t)
        
    for msg, t in list(pcl_queue.queue):
        bag.write(pcl_topic, msg, t)
    
    bag.flush()
    bag.close()
    print('*** Bag Write {} Done! ***'.format(filename))


auto_capture = False
def autostate_callback(msg):
    global pose_queue
    global obs_queue
    global image_queue
    global last_autostate
    
    # print('*** autostate {}, last {} ***'.format(msg.CurDriveMode, last_autostate))
    if msg.CurDriveMode == 0 and last_autostate == 2:
        global auto_capture
        if auto_capture:
            start_capture(auto_queue, pose_queue, obs_queue, image_queue)
        global take_over_count
        take_over_count = take_over_count + 1
    last_autostate = msg.CurDriveMode
    # 
    if not auto_queue.full():
        auto_queue.put((msg, rospy.Time.now()))
    else:
        auto_queue.get()


def ego_pose_callback(msg):
    if not pose_queue.full():
        pose_queue.put((msg, rospy.Time.now()))
    else:
        pose_queue.get()

    global last_point
    point = (msg.state.pose.pose.position.x,
            msg.state.pose.pose.position.y)
    if last_point is None:
        last_point = point

    vb = np.array([point[0], point[1]])
    va = np.array([last_point[0], last_point[1]])
    gap = np.linalg.norm(vb - va)
    # print('*** gap {} ***'.format(gap))
    last_point = point
    
    global total_distance
    global last_autostate
    if last_autostate == 2:
        total_distance = total_distance + gap
    # print('++++ ego len ', len(pose_queue.queue))


def obstacles_callback(msg):
    if not obs_queue.full():
        obs_queue.put((msg, rospy.Time.now()))
    else:
        obs_queue.get()
    # print('++++ obs len ', len(obs_queue.queue))


def image_callback(msg):
    if not image_queue.full():
        image_queue.put((msg, rospy.Time.now()))
    else:
        image_queue.get()
    # print('++++ img len ', len(image_queue.queue))


def pcl_callback(msg):
    if not pcl_queue.full():
        pcl_queue.put((msg, rospy.Time.now()))
    else:
        pcl_queue.get()
    # print('++++ pcl len ', len(pcl_queue.queue))


ros_main_thread_pid = -1
def ros_main_thread():
    
    rospy.loginfo("*** Ros Main Thread ID %d", os.getpid())
    global ros_main_thread_pid
    ros_main_thread_pid = os.getpid()
    
    try:
        global auto_topic
        global ego_pose_topic
        global obs_topic
        global left_cam_topic

        rospy.Subscriber(auto_topic, AutoStateEx, autostate_callback, queue_size=20)
        rospy.Subscriber(left_cam_topic, CompressedImage, image_callback, queue_size=10)
        rospy.Subscriber(ego_pose_topic, RigidBodyStateStamped, ego_pose_callback, queue_size=100)
        rospy.Subscriber(obs_topic, TrackingBoxArray, obstacles_callback, queue_size=10)
        rospy.Subscriber(pcl_topic, PointCloud2, pcl_callback, queue_size=10)
        # rospy.Subscriber(pcl_topic, )

        global traffic_publisher
        traffic_publisher = rospy.Publisher(traffic_light_topic, DetectionBoxArray, queue_size=1)

        rospy.loginfo("*** Create Subscribers Done, Start Loop ***")
        rospy.spin()
    finally:
        rospy.loginfo("*** All Done ***")
        

class MyViz(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.initialize()
        self.setWindowTitle("bag-capture")
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()        
        h_layout = QHBoxLayout()
        
        capture_button = QPushButton("Capture")
        capture_button.clicked.connect(self.onCaptureButtonClick)
        h_layout.addWidget(capture_button)
        
        red_button = QPushButton("RedSignal")
        red_button.clicked.connect( self.onRedSignalButtonClick)
        h_layout.addWidget( red_button )
        
        green_button = QPushButton("GreenSignal")
        green_button.clicked.connect(self.onGreenSignalButtonClick)
        h_layout.addWidget(green_button)
        
        layout.addLayout(h_layout)
        self.setLayout(layout)

    ## GUI button event handling
    def onCaptureButtonClick(self):
        global auto_queue, pose_queue, obs_queue, image_queue
        start_capture(auto_queue, pose_queue, obs_queue, image_queue, pcl_queue)
        # QMessageBox.about(self,"消息框标题","这是关于软件的说明",QMessageBox.Yes | QMessageBox.No)
        print("*** Capture Done ***")
        
        
    def onRedSignalButtonClick(self):
        traffic_light_detection = DetectionBoxArray()
        traffic_light = DetectionBox()
        traffic_light.signal.flags = ObjectSignals.TRAFFIC_LIGHT_RED
        traffic_light_detection.detections.append(traffic_light)
        global traffic_publisher
        traffic_publisher.publish(traffic_light_detection)
        print('*** Send RED Signal...')
    
        
    def onGreenSignalButtonClick(self):
        traffic_light_detection = DetectionBoxArray()
        traffic_light = DetectionBox()
        traffic_light.signal.flags = ObjectSignals.TRAFFIC_LIGHT_GREEN
        traffic_light_detection.detections.append(traffic_light)
        global traffic_publisher
        traffic_publisher.publish(traffic_light_detection)
        print('*** Send GREEN Signal...')
        
    
## Start the Application
if __name__ == '__main__':
    # ros comm
    rospy.init_node("Bug-Capture-Node", anonymous=True)
    ros_thread = threading.Thread(
        target = ros_main_thread, args=())
    ros_thread.setDaemon(True)
    ros_thread.start()
    
    # qt-gui
    app = QApplication(sys.argv)
    myviz = MyViz()
    myviz.resize(200, 20)
    myviz.show()
    app.exec_()
    global total_distance, take_over_count
    print('### Total Distance - {} km, take over {} times'.format(total_distance / 1000.0, take_over_count))
    # kill ros_main_thread
    global ros_main_thread_pid
    time.sleep(5)
    os.kill(ros_main_thread_pid, signal.SIGTERM)
    print('### kill ros_main_thread {} !!!'.format(ros_main_thread_pid))

