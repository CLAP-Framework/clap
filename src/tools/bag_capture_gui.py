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
auto_topic = '/xp/auto_state_ex'
left_cam_topic = '/left_usb_cam/image_raw/compressed'
ego_pose_topic = '/zzz/navigation/ego_pose'
obs_topic  = '/zzz/perception/objects_tracked'

traffic_publisher = None
traffic_light_topic = "/zzz/perception/traffic_lights"

# 2-autopilot, 0-manually
last_autostate = 0
# bag record for 5 minutes
window_seconds = 60 * 5

last_point = None
total_distance = 0.0

# ego_pose hz 100
pose_queue  = Queue(100 * window_seconds)
# perception obstacle hz 10
obs_queue   = Queue(10 * window_seconds)
# usb cam hz 10
image_queue = Queue(10 * window_seconds) 


def start_capture(pose_queue, obs_queue, image_queue):
    t = threading.Thread(
        target = write2bag, 
        args = (pose_queue, obs_queue, image_queue))
    t.setDaemon(False)
    t.start()
    rospy.loginfo("*** launch thread record bag file ###")
    

def write2bag(pose_queue, obs_queue, image_queue):
    filename = time.strftime("%Y-%m-%d_%H:%M:%S.bag", time.localtime()) 
    # time.sleep(60)
    bag = rosbag.Bag(filename, "w")
    for msg in list(pose_queue.queue):
        bag.write(ego_pose_topic, msg)

    for msg in list(obs_queue.queue):
        bag.write(obs_topic, msg)

    for msg in list(image_queue.queue):
        bag.write(left_cam_topic, msg)
    
    bag.close()
    print('*** bag write {} done! ***'.format(filename))



def autostate_callback(msg):
    global pose_queue
    global obs_queue
    global image_queue
    global last_autostate
    # print('*** autostate {}, last {} ***'.format(msg.CurDriveMode, last_autostate))
    if msg.CurDriveMode == 0 and last_autostate == 2:
        start_capture(pose_queue, obs_queue, image_queue)
    last_autostate = msg.CurDriveMode


def ego_pose_callback(msg):
    if not pose_queue.full():
        pose_queue.put(msg)
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
    global total_distance
    total_distance = total_distance + gap
    # print('++++ ego len ', len(pose_queue.queue))


def obstacles_callback(msg):
    if not obs_queue.full():
        obs_queue.put(msg)
    else:
        obs_queue.get()
    # print('++++ obs len ', len(obs_queue.queue))


def image_callback(msg):
    if not image_queue.full():
        image_queue.put(msg)
    else:
        image_queue.get()
    # print('++++ img len ', len(image_queue.queue))

ros_main_thread_pid = -1
def ros_main_thread():
    rospy.loginfo("*** ros main thread pid %d", os.getpid())
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

        global traffic_publisher
        traffic_publisher = rospy.Publisher(traffic_light_topic, DetectionBoxArray, queue_size=1)

        rospy.loginfo("create Subscribers done, start loop...")
        rospy.spin()
    finally:
        rospy.loginfo("*** all done ***")
        

class MyViz(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.initialize()
        self.setWindowTitle("bag-capture")
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

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
        global pose_queue, obs_queue, image_queue
        start_capture(pose_queue, obs_queue, image_queue)
        print("handle capture done!")
        
        
    def onRedSignalButtonClick(self):
        traffic_light_detection = DetectionBoxArray()
        traffic_light = DetectionBox()
        traffic_light.signal.flags = ObjectSignals.TRAFFIC_LIGHT_RED
        traffic_light_detection.detections.append(traffic_light)
        global traffic_publisher
        traffic_publisher.publish(traffic_light_detection)
        print('send red signal...')
    
        
    def onGreenSignalButtonClick(self):
        traffic_light_detection = DetectionBoxArray()
        traffic_light = DetectionBox()
        traffic_light.signal.flags = ObjectSignals.TRAFFIC_LIGHT_GREEN
        traffic_light_detection.detections.append(traffic_light)
        global traffic_publisher
        traffic_publisher.publish(traffic_light_detection)
        print('send green signal...')
        
    
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
    global total_distance
    print('### total distance - {} km'.format(total_distance / 1000.0))
    # kill ros_main_thread
    global ros_main_thread_pid
    os.kill(ros_main_thread_pid, signal.SIGTERM)

