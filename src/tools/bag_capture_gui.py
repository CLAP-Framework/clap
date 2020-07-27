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
from nav_msgs.msg import Path
from cv_bridge import CvBridgeError, CvBridge
from visualization_msgs.msg import Marker, MarkerArray
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
ego_pose_topic  = '/zzz/navigation/ego_pose'
obs_topic = '/zzz/perception/objects_tracked'
left_cam_topic  = '/left_usb_cam/image_raw/compressed'
pcl_topic = '/middle/rslidar_points'
ego_marker_topic = '/zzz/cognition/ego_markerarray'
lanes_marker_topic = '/zzz/cognition/lanes_markerarray'
obstacles_marker_topic = '/zzz/cognition/obstacles_markerarray'
sent_ref_path_topic = '/zzz/cognition/sent_ref_path'
all_trajectory_path_topic = '/zzz/planning/all_trajectory_path'
decision_trajectory_path_topic = '/zzz/planning/decision_trajectory_path'
prepoint_topic = '/pre_point'
collision_topic = '/zzz/planning/collision_circle'


traffic_publisher = None
traffic_light_topic = "/zzz/perception/traffic_lights"

# 2-autopilot, 0-manually
last_autostate  = 0
take_over_count = 0

# bag record default 1 minutes
window_seconds = 60

last_point = None
total_distance = 0.0

# all topics
# autostate_mode, 50 hz
auto_queue = Queue(50 * window_seconds)
# ego_pose hz 100
pose_queue = Queue(100 * window_seconds)
# perception obstacle hz 10
obs_queue = Queue(10 * window_seconds)
# usb cam hz 10
image_queue = Queue(10 * window_seconds) 
# pcl hz 10
pcl_queue = Queue(10 * window_seconds)
# 5 hz
ego_marker_queue = Queue(5 * window_seconds)
# 5 hz
lanes_marker_queue = Queue(5 * window_seconds)
# 5 hz
obstacles_marker_queue = Queue(5 * window_seconds)
# 20 hz
sent_ref_path_queue = Queue(20 * window_seconds)
# 5 hz
all_trajectory_path_queue = Queue(5 * window_seconds)
# 5 hz
decision_trajectory_path_queue = Queue(5 * window_seconds)
# 20 hz
prepoint_queue = Queue(20 * window_seconds)
# 5 hz
collision_queue = Queue(5 * window_seconds)


global_topic_queue_pairs = [
    (auto_topic, auto_queue),
    (ego_pose_topic, pose_queue), 
    (obs_topic, obs_queue),
    (left_cam_topic, image_queue),
    (pcl_topic, pcl_queue),
    (ego_marker_topic, ego_marker_queue),
    (lanes_marker_topic, lanes_marker_queue),
    (obstacles_marker_topic, obstacles_marker_queue),
    (sent_ref_path_topic, sent_ref_path_queue),
    (all_trajectory_path_topic, all_trajectory_path_queue),
    (decision_trajectory_path_topic, decision_trajectory_path_queue),
    (prepoint_topic, prepoint_queue),
    (collision_topic, collision_queue)]


def start_capture(topic_queue_pairs):
    t = threading.Thread(target=write2bag, args=(topic_queue_pairs, ))
    t.setDaemon(False)
    t.start()
    rospy.loginfo("### Launch Thread Record Bag File! ###")
    

def write2bag(topic_queue_pairs):

    file_name = time.strftime("%Y-%m-%d_%H-%M-%S.bag", time.localtime()) 
    # time.sleep(60)
    bag = rosbag.Bag(file_name, "w")
    for topic, que in topic_queue_pairs:
        for msg, t in list(que.queue):
            bag.write(topic, msg, t)
    
    bag.flush()
    bag.close()
    print('### Bag Write {} Done! ###'.format(file_name))


auto_capture = False
def autostate_callback(msg):
    global pose_queue, obs_queue, image_queue, last_autostate
    
    # print('*** autostate {}, last {} ***'.format(msg.CurDriveMode, last_autostate))
    if msg.CurDriveMode == 0 and last_autostate == 2:
        global auto_capture, global_topic_queue_pairs, take_over_count
        if auto_capture:
            start_capture(global_topic_queue_pairs)

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
    
    global total_distance, last_autostate
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


def ego_marker_callback(msg):
    global ego_marker_queue
    if not ego_marker_queue.full():
        ego_marker_queue.put((msg, rospy.Time.now()))
    else:
        ego_marker_queue.get()



def lanes_marker_callback(msg):
    global lanes_marker_queue
    if not lanes_marker_queue.full():
        lanes_marker_queue.put((msg, rospy.Time.now()))
    else:
        lanes_marker_queue.get()



def obstacles_marker_callback(msg):
    global obstacles_marker_queue
    if not obstacles_marker_queue.full():
        obstacles_marker_queue.put((msg, rospy.Time.now()))
    else:
        obstacles_marker_queue.get()



def sent_ref_path_callback(msg):
    global sent_ref_path_queue
    if not sent_ref_path_queue.full():
        sent_ref_path_queue.put((msg, rospy.Time.now()))
    else:
        sent_ref_path_queue.get()
    


def all_trajectory_path_callback(msg):
    global all_trajectory_path_queue
    if not all_trajectory_path_queue.full():
        all_trajectory_path_queue.put((msg, rospy.Time.now()))
    else:
        all_trajectory_path_queue.get()



def decision_trajectory_path_callback(msg):
    global decision_trajectory_path_queue
    if not decision_trajectory_path_queue.full():
        decision_trajectory_path_queue.put((msg, rospy.Time.now()))
    else:
        decision_trajectory_path_queue.get()


    
def prepoint_callback(msg):
    global prepoint_queue
    if not prepoint_queue.full():
        prepoint_queue.put((msg, rospy.Time.now()))
    else:
        prepoint_queue.get()



def collision_callback(msg):
    global collision_queue
    if not collision_queue.full():
        collision_queue.put((msg, rospy.Time.now()))
    else:
        collision_queue.get()



ros_main_thread_pid = -1

def ros_main_thread():
    
    rospy.loginfo("### Ros Main Thread ID %d ###", os.getpid())
    global ros_main_thread_pid
    ros_main_thread_pid = os.getpid()
    
    try:
        rospy.Subscriber(auto_topic, AutoStateEx, autostate_callback, queue_size=20)
        rospy.Subscriber(left_cam_topic, CompressedImage, image_callback, queue_size=10)
        rospy.Subscriber(ego_pose_topic, RigidBodyStateStamped, ego_pose_callback, queue_size=100)
        rospy.Subscriber(obs_topic, TrackingBoxArray, obstacles_callback, queue_size=10)
        rospy.Subscriber(pcl_topic, PointCloud2, pcl_callback, queue_size=10)

        rospy.Subscriber(ego_marker_topic, MarkerArray, ego_marker_callback, queue_size=5)
        rospy.Subscriber(lanes_marker_topic, MarkerArray, lanes_marker_callback, queue_size=5)
        rospy.Subscriber(obstacles_marker_topic, MarkerArray, obstacles_marker_callback, queue_size=10)

        rospy.Subscriber(sent_ref_path_topic, Path, sent_ref_path_callback, queue_size=20)
        
        rospy.Subscriber(all_trajectory_path_topic, MarkerArray, all_trajectory_path_callback, queue_size=5)
        rospy.Subscriber(decision_trajectory_path_topic, Path, decision_trajectory_path_callback, queue_size=5)
        rospy.Subscriber(prepoint_topic, Marker, prepoint_callback, queue_size=20)
        rospy.Subscriber(collision_topic, MarkerArray, collision_callback, queue_size=5)

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
        h_layout.addWidget(red_button)
        
        green_button = QPushButton("GreenSignal")
        green_button.clicked.connect(self.onGreenSignalButtonClick)
        h_layout.addWidget(green_button)
        
        layout.addLayout(h_layout)
        self.setLayout(layout)

    ## GUI button event handling
    def onCaptureButtonClick(self):
        global global_topic_queue_pairs
        start_capture(global_topic_queue_pairs)
        # QMessageBox.about(self,"消息框标题","这是关于软件的说明",QMessageBox.Yes | QMessageBox.No)
        print("### Capture Done! ###")
        
        
    def onRedSignalButtonClick(self):
        traffic_light_detection = DetectionBoxArray()
        traffic_light = DetectionBox()
        traffic_light.signal.flags = ObjectSignals.TRAFFIC_LIGHT_RED
        traffic_light_detection.detections.append(traffic_light)
        global traffic_publisher
        traffic_publisher.publish(traffic_light_detection)
        print('### Send RED Signal...')
    
        
    def onGreenSignalButtonClick(self):
        traffic_light_detection = DetectionBoxArray()
        traffic_light = DetectionBox()
        traffic_light.signal.flags = ObjectSignals.TRAFFIC_LIGHT_GREEN
        traffic_light_detection.detections.append(traffic_light)
        global traffic_publisher
        traffic_publisher.publish(traffic_light_detection)
        print('### Send GREEN Signal...')
        
    
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
    print('### Total Distance - {} km, Take over {} times ###'.format(total_distance / 1000.0, take_over_count))
    # kill ros_main_thread
    global ros_main_thread_pid
    time.sleep(5)
    os.kill(ros_main_thread_pid, signal.SIGTERM)
    print('### kill ros_main_thread {} ! ###'.format(ros_main_thread_pid))

