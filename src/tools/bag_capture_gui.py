#!/usr/bin/env python
# coding: utf-8
import os
import sys
import time
import datetime
import threading
import copy
import signal
import roslib
import rospy
import copy
import rosbag
#import subprocess32

from sensor_msgs.msg import Image, CompressedImage, PointCloud2, NavSatFix, Imu
from nav_msgs.msg import Path, Odometry

from cv_bridge import CvBridgeError, CvBridge
from visualization_msgs.msg import Marker, MarkerArray

# from rslidar_msgs.msg import rslidarPacket
# from rslidar_msgs.msg import rslidarScan
from tf2_msgs.msg import TFMessage 

from Queue import Queue
import argparse
import utm
import numpy as np

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rviz

# zzz message reference
from xpmotors_can_msgs.msg import AutoStateEx, ESCStatus, EPSStatus, AutoCtlReq
from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_planning_msgs.msg import DecisionTrajectory
from zzz_perception_msgs.msg import TrackingBox, TrackingBoxArray, DetectionBox, DetectionBoxArray, ObjectSignals

roslib.load_manifest('rosbag')

# parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
# parser.add_argument('bagfile', nargs=1, help='input bag file')
# parser.add_argument('--window_seconds', nargs=1, help='max time offset (sec) to correct.', default='120', type=int)
# args = parser.parse_args()

traffic_publisher = None
traffic_light_topic = "/zzz/perception/traffic_lights"
real_velocity_topic = "/xp/esc_status"
desired_velocity_topic = "/zzz/planning/decision_trajectory"

# 2-autopilot, 0-manually
last_autostate  = 0
take_over_count = 0

#current bag number and non-problem take over
current_bag_number = 1
non_problem_take_over = 0
current_time = time.time()

# bag record default 1 minutes
window_seconds = 60*1.5

last_point = None
total_distance = 0.0

# all topics
auto_topic = '/xp/auto_state_ex'
esc_topic = '/xp/esc_status'
eps_topic = '/xp/eps_status'
auto_ctl_topic = '/xp/auto_control'

obs_topic = '/zzz/perception/objects_tracked'
left_cam_topic  = '/left_usb_cam/image_raw/compressed'
gps_fix_topic = '/localization/gps/fix'
imu_dat_topic = '/localization/imu/data'
ego_pose_topic  = '/zzz/navigation/ego_pose'
odom_topic = '/localization/gps/odom'

pcl_topic = '/middle/rslidar_points'
pcl_packets_topic = '/middle/rslidar_packets' 
pcl_packets_difop_topic = '/middle/rslidar_packets_difop'

ego_marker_topic = '/zzz/cognition/ego_markerarray'
lanes_marker_topic = '/zzz/cognition/lanes_markerarray'
obstacles_marker_topic = '/zzz/cognition/obstacles_markerarray'
sent_ref_path_topic = '/zzz/cognition/sent_ref_path'
all_trajectory_path_topic = '/zzz/planning/all_trajectory_path'
decision_trajectory_path_topic = '/zzz/planning/decision_trajectory_path'
prepoint_topic = '/pre_point'
collision_topic = '/zzz/planning/collision_circle'
tf_topic = '/tf'

# queue
# autostate_mode, 50 hz
auto_queue = Queue(50 * window_seconds)
esc_queue = Queue(50 * window_seconds)
eps_queue = Queue(50 * window_seconds)
auto_ctl_queue = Queue(50 * window_seconds)

# ego_pose hz 100
pose_queue = Queue(100 * window_seconds)
odom_queue = Queue(100 * window_seconds)
gpsfix_queue = Queue(100 * window_seconds)
imudat_queue = Queue(100 * window_seconds)
# perception obstacle hz 10
obs_queue = Queue(10 * window_seconds)
# usb cam hz 10
image_queue = Queue(10 * window_seconds)
# pcl hz 10
pcl_queue = Queue(10 * window_seconds)
pcl_packets_difop_queue = Queue(10 * window_seconds)
pcl_packets_queue = Queue(10 * window_seconds)
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
# 100 hz
tf_queue = Queue(100 * window_seconds)


def start_capture(topic_queue_pairs):
    t = threading.Thread(target=write2bag, args=(topic_queue_pairs, ))
    t.setDaemon(False)
    t.start()
    rospy.loginfo("### Launch Thread Record Bag File! ###")
    

def write2bag(topic_queue_pairs):

    global current_bag_number
    global current_time

    click_time = current_time

    file_name = time.strftime("%Y-%m-%d_%H-%M-%S.bag", time.localtime())
    file_name = str(current_bag_number) + '_' + file_name
    # time.sleep(60)
    bag = rosbag.Bag(file_name, "w")
    for topic, que, _, _, _ in topic_queue_pairs:
        for msg, t, record_time in list(que.queue):
            if click_time - record_time < window_seconds:
                bag.write(topic, msg, t)
            else:
                break
    
    bag.flush()
    bag.close()
    print('### Bag Write {} Done! ###'.format(file_name))


auto_capture = False
def autostate_callback(msg):
    global pose_queue, obs_queue, image_queue, last_autostate, current_time
    current_time = time.time()
    
    if msg.CurDriveMode == 0 and last_autostate == 2:
        global auto_capture, global_topic_queue_pairs, take_over_count
        if auto_capture:
            start_capture(global_topic_queue_pairs)

        take_over_count = take_over_count + 1

    last_autostate = msg.CurDriveMode
    if not auto_queue.full():
        auto_queue.put((msg, msg.header.stamp, current_time))
    else:
        auto_queue.get()


def auto_ctl_callback(msg):
    global current_time
    if not auto_ctl_queue.full():
        auto_ctl_queue.put((msg, msg.header.stamp, current_time))
    else:
        auto_ctl_queue.get()


def esc_callback(msg):
    global current_time
    if not esc_queue.full():
        esc_queue.put((msg, msg.header.stamp, current_time))
    else:
        esc_queue.get()


def eps_callback(msg):
    global current_time
    if not eps_queue.full():
        eps_queue.put((msg, msg.header.stamp, current_time))
    else:
        eps_queue.get()


def ego_pose_callback(msg):
    global current_time
    if not pose_queue.full():
        pose_queue.put((msg, msg.header.stamp, current_time))
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
    last_point = point
    
    global total_distance, last_autostate
    if last_autostate == 2:
        total_distance = total_distance + gap


def odom_callback(msg):
    global current_time
    if not odom_queue.full():
        odom_queue.put((msg, msg.header.stamp, current_time))
    else:
        odom_queue.get()


def gpsfix_callback(msg):
    global current_time
    if not gpsfix_queue.full():
        gpsfix_queue.put((msg, msg.header.stamp, current_time))
    else:
        gpsfix_queue.get()


def imudat_callback(msg):
    global current_time
    if not imudat_queue.full():
        imudat_queue.put((msg, msg.header.stamp, current_time))
    else:
        imudat_queue.get()


def obstacles_callback(msg):
    global current_time
    if not obs_queue.full():
        obs_queue.put((msg, msg.header.stamp, current_time))
    else:
        obs_queue.get()


def image_callback(msg):
    global current_time
    if not image_queue.full():
        image_queue.put((msg, msg.header.stamp, current_time))
    else:
        image_queue.get()


def pcl_callback(msg):
    global current_time
    if not pcl_queue.full():
        pcl_queue.put((msg, msg.header.stamp, current_time))
    else:
        pcl_queue.get()


def pcl_packets_callback(msg):
    global current_time
    if not pcl_packets_queue.full():
        pcl_packets_queue.put((msg, msg.header.stamp, current_time))
    else:
        pcl_packets_queue.get()


def pcl_packets_difop_callback(msg):
    global current_time
    if not pcl_packets_difop_queue.full():
        pcl_packets_difop_queue.put((msg, msg.header.stamp, current_time))
    else:
        pcl_packets_difop_queue.get()


def ego_marker_callback(msg):
    global current_time
    global ego_marker_queue
    if not ego_marker_queue.full():
        ego_marker_queue.put((msg, rospy.Time.now(), current_time))
    else:
        ego_marker_queue.get()


def lanes_marker_callback(msg):
    global current_time
    global lanes_marker_queue
    if not lanes_marker_queue.full():
        lanes_marker_queue.put((msg, rospy.Time.now(), current_time))
    else:
        lanes_marker_queue.get()


def obstacles_marker_callback(msg):
    global current_time
    global obstacles_marker_queue
    if not obstacles_marker_queue.full():
        obstacles_marker_queue.put((msg, rospy.Time.now(), current_time))
    else:
        obstacles_marker_queue.get()


def sent_ref_path_callback(msg):
    global current_time
    global sent_ref_path_queue
    if not sent_ref_path_queue.full():
        sent_ref_path_queue.put((msg, msg.header.stamp, current_time))
    else:
        sent_ref_path_queue.get()
    

def all_trajectory_path_callback(msg):
    global current_time
    global all_trajectory_path_queue
    if not all_trajectory_path_queue.full():
        all_trajectory_path_queue.put((msg, rospy.Time.now(), current_time))
    else:
        all_trajectory_path_queue.get()


def decision_trajectory_path_callback(msg):
    global current_time
    global decision_trajectory_path_queue
    if not decision_trajectory_path_queue.full():
        decision_trajectory_path_queue.put((msg, msg.header.stamp, current_time))
    else:
        decision_trajectory_path_queue.get()


def prepoint_callback(msg):
    global current_time
    global prepoint_queue
    if not prepoint_queue.full():
        prepoint_queue.put((msg, msg.header.stamp, current_time))
    else:
        prepoint_queue.get()


def collision_callback(msg):
    global current_time
    global collision_queue
    if not collision_queue.full():
        collision_queue.put((msg, rospy.Time.now(), current_time))
    else:
        collision_queue.get()


def tf_callback(msg):
    global current_time
    global tf_queue
    if not tf_queue.full():
        tf_queue.put((msg, msg.transforms[0].header.stamp, current_time))
    else:
        tf_queue.get()

real_velocity_value = 0.0
desired_velocity_value = 0.0

def desired_velocity_callback(msg):
    global desired_velocity_value
    desired_velocity_value = msg.desired_speed
        #self.desired_velocity.setText(str(msg.desired_speed))
    
def real_velocity_callback(msg):
    global real_velocity_value
    real_velocity_value = msg.RRWheelSpd
        #data_buffer = copy.deepcopy(msg.RRWheelSpd)

global_topic_queue_pairs = [
    ###  (topic, queue, class, callback, hz)
    (auto_topic, auto_queue, AutoStateEx, autostate_callback, 50),
    (auto_ctl_topic, auto_ctl_queue, AutoCtlReq, auto_ctl_callback, 50),
    (eps_topic, eps_queue, EPSStatus, eps_callback, 50),
    (esc_topic, esc_queue, ESCStatus, esc_callback, 50),
    (ego_pose_topic, pose_queue, RigidBodyStateStamped, ego_pose_callback, 100),
    (odom_topic, odom_queue, Odometry, odom_callback, 100),
    (gps_fix_topic, gpsfix_queue, NavSatFix, gpsfix_callback, 100),
    (imu_dat_topic, imudat_queue, Imu, imudat_callback, 100),
    (obs_topic, obs_queue, TrackingBoxArray, obstacles_callback, 10),
    (left_cam_topic, image_queue, CompressedImage, image_callback, 10),
    (pcl_topic, pcl_queue, PointCloud2, pcl_callback, 10),
    # (pcl_packets_topic, pcl_packets_queue, rslidarScan, pcl_packets_callback, 10),
    # (pcl_packets_difop_topic, pcl_packets_difop_queue, rslidarPacket, pcl_packets_difop_callback, 10),
    (ego_marker_topic, ego_marker_queue, MarkerArray, ego_marker_callback, 5),
    (lanes_marker_topic, lanes_marker_queue, MarkerArray, lanes_marker_callback, 5),
    (obstacles_marker_topic, obstacles_marker_queue, MarkerArray, obstacles_marker_callback, 10),
    (sent_ref_path_topic, sent_ref_path_queue, Path, sent_ref_path_callback, 20),
    # (all_trajectory_path_topic, all_trajectory_path_queue, MarkerArray, all_trajectory_path_callback, 5),
    (decision_trajectory_path_topic, decision_trajectory_path_queue, Path, decision_trajectory_path_callback, 5),
    (prepoint_topic, prepoint_queue, Marker, prepoint_callback, 100),
    (collision_topic, collision_queue, MarkerArray, collision_callback, 5), 
    (tf_topic, tf_queue, TFMessage, tf_callback, 100)]


ros_main_thread_pid = -1

def ros_main_thread():
    
    rospy.loginfo("### Ros Main Thread ID %d ###", os.getpid())
    global ros_main_thread_pid
    ros_main_thread_pid = os.getpid()
    
    try:
        # batch create sub
        for t, _, cls, cb, hz in global_topic_queue_pairs:
            rospy.Subscriber(t, cls, cb, queue_size=hz)
        
        global traffic_publisher
        traffic_publisher = rospy.Publisher(traffic_light_topic, DetectionBoxArray, queue_size=10)
        real_velocity_subscriber = rospy.Subscriber(real_velocity_topic, ESCStatus, real_velocity_callback)
        desired_velocity_subscriber = rospy.Subscriber(desired_velocity_topic, DecisionTrajectory, desired_velocity_callback)
        


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
        self.statusBar = QStatusBar()
        # record number of current bag
        
        self.timer = QTimer()
        
        self.timer.timeout.connect(self.desired_velocity_ui)
        self.timer.timeout.connect(self.real_velocity_ui)
        self.timer.start(200)


        # self.real_velocity_subscriber = rospy.Subscriber(real_velocity_topic, ESCStatus, self.real_velocity_callback)
        # self.desired_velocity_subscriber = rospy.Subscriber(desired_velocity_topic, DecisionTrajectory, self.desired_velocity_callback)
        
        self.manager = self.frame.getManager()
        ## Here we create the layout and other widgets in the usual Qt way.
        global_Hlayout = QHBoxLayout()
        layout = QVBoxLayout()
        save_layout = QHBoxLayout()
        traffic_signal_layout = QHBoxLayout()
        
        capture_question_button = QPushButton("BAD BAG")
        capture_question_button.setFixedSize(150,40)
        capture_question_button.setStyleSheet("color:red")
        capture_question_button.clicked.connect(self.onCaptureQuestionButtonClick)
        save_layout.addWidget(capture_question_button)
        
        capture_showcase_button = QPushButton("GOOD BAG")
        capture_showcase_button.setFixedSize(100,40)
        capture_showcase_button.setStyleSheet("color:green")
        capture_showcase_button.clicked.connect(self.onCaptureShowcaseButtonClick)
        save_layout.addWidget(capture_showcase_button)
        
        red_button = QPushButton("RedSignal")
        red_button.setStyleSheet("background:red")
        red_button.clicked.connect( self.onRedSignalButtonClick)
        traffic_signal_layout.addWidget(red_button)
        
        green_button = QPushButton("GreenSignal")
        green_button.setStyleSheet("background:green")
        green_button.clicked.connect(self.onGreenSignalButtonClick)
        traffic_signal_layout.addWidget(green_button)

        option_layout = QVBoxLayout()
        
        normal_take_over = QPushButton("Normal take-over")
        normal_take_over.setFixedSize(150, 40)
        normal_take_over.clicked.connect(self.count_normal_take_over)
        
        launch_system_button = QPushButton("LAUNCH SYSTEM")
        launch_system_button.clicked.connect(self.launch_system)
        
        relaunch_system_button = QPushButton("REBOOST STSTEM")
        relaunch_system_button.clicked.connect(self.relaunch_system)

        option_layout.addWidget(launch_system_button)
        option_layout.addWidget(relaunch_system_button)
        option_layout.addWidget(normal_take_over)
        
        info_layout = QVBoxLayout()

        self.font = QFont()
        self.font.setPointSize(18)

        desired_velocity_label = QLabel('Desired Velocity:')
        self.desired_velocity = QTextEdit()
        self.desired_velocity.setFixedSize(100,40)
        self.desired_velocity.setFont(self.font)
        self.desired_velocity.setText('0')
        real_velocity_label = QLabel('Real Velocity:')
        self.real_velocity = QTextEdit()
        self.real_velocity.setFixedSize(100, 40)
        self.real_velocity.setFont(self.font)
        self.real_velocity.setText('0')

        info_layout.addWidget(desired_velocity_label)
        info_layout.addWidget(self.desired_velocity)
        info_layout.addWidget(real_velocity_label)
        info_layout.addWidget(self.real_velocity)

        # ADD a case description 
        self.case_description = QTextEdit()
        layout.addWidget(self.case_description)
        
        layout.addLayout(save_layout)
        layout.addLayout(traffic_signal_layout)
        layout.addWidget(self.statusBar)
        global_Hlayout.addLayout(layout)
        global_Hlayout.addLayout(option_layout)
        global_Hlayout.addLayout(info_layout)
        self.setLayout(global_Hlayout)

        self.statusBar.showMessage('Initialize Successfully')
    ## GUI button event handling
    def onCaptureQuestionButtonClick(self):
        global global_topic_queue_pairs, current_bag_number
        start_capture(global_topic_queue_pairs)
        with open('Record.txt', 'a+') as f:
            f.write('BAG NO.{0} | Question bag | {1}'.format(str(current_bag_number), self.case_description.toPlainText()) + ' | ' + str(rospy.Time.now()) + '\n')
        self.case_description.clear()
        self.statusBar.showMessage('No.{0} bag captured'.format(str(current_bag_number)), 5000)
        current_bag_number += 1
        print("### Capture Done! ###")
    
    def onCaptureShowcaseButtonClick(self):
        global global_topic_queue_pairs, current_bag_number
        start_capture(global_topic_queue_pairs)
        with open('Record.txt', 'a+') as f:
            f.write('BAG NO.{0} | ShowCase bag | {1}'.format(str(current_bag_number), self.case_description.toPlainText()) + ' | ' + str(rospy.Time.now()) +'\n')
        self.case_description.clear()
        self.statusBar.showMessage('No.{0} bag captured'.format(str(current_bag_number)), 5000)
        current_bag_number += 1
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
        
    def count_normal_take_over(self):
        global non_problem_take_over
        non_problem_take_over += 1

    def real_velocity_ui(self):
        global real_velocity_value
        self.real_velocity.setText(str(real_velocity_value/3.6))
        
    def desired_velocity_ui(self):
        global desired_velocity_value
        self.desired_velocity.setText(str(desired_velocity_value))
        

    def launch_system(self):
        pass
        #self.load_path_cmd = subprocess32.Popen(['/bin/bash', '-i', '-c', 'cd /home/novauto/CLAP/zzz&&./load_ref_path.sh'], start_new_session=True)
        # self.load_main_cmd = subprocess32.Popen(['/bin/bash', '-i', '-c', 'cd /home/novauto/CLAP/zzz&&./load_main.sh'], start_new_session=True)
        # print('CMD PID IS !!!!!!!!!{0}'.format(self.load_main_cmd.pid))

    def closeEvent(self, event):
        try:
            #os.kill(self.load_path_cmd.pid, signal.SIGKILL)
            # os.kill(self.load_main_cmd.pid, signal.SIGKILL)
            print('All process killed!!!!')
        except:
            pass


    def relaunch_system(self):
        print('a')


## Start the Application
if __name__ == '__main__':
    app = QApplication(sys.argv)
    myviz = MyViz()
    # ros comm
    rospy.init_node("Bug-Capture-Node", anonymous=True)

    ros_thread = threading.Thread(
        target = ros_main_thread, args=())
    ros_thread.setDaemon(True)
    ros_thread.start()
    
    # open a new text file to record testing result
    with open('Record.txt', 'a+') as f:
        f.write('Today is :{0}\n'.format(datetime.date.today()))
        f.write('This testing begin from:{0}\n'.format(datetime.datetime.now().strftime('%H:%M:%S')))
        f.write('---------------------------\n')
    # qt-gui
    
    myviz.resize(250, 150)
    myviz.show()
    app.exec_()
    global total_distance, take_over_count, non_problem_take_over
    print('### Total Distance - {} km, Take over {} times ###'.format(total_distance / 1000.0, take_over_count))
    # kill ros_main_thread
    with open('Record.txt', 'a+') as f:
        f.write('---------------------------\n')
        f.write('This testing end at:{0}\n'.format(datetime.datetime.now().strftime('%H:%M:%S')))
        f.write('Today we droved {}km, Take over {} times'.format(total_distance / 1000.0, take_over_count - non_problem_take_over) + '\n')
    
    global ros_main_thread_pid
    os.kill(ros_main_thread_pid, signal.SIGINT)
    time.sleep(5)
    
    cmd = "kill -9 %d" % int(ros_main_thread_pid)
    os.system(cmd)
    # os.kill(ros_main_thread_pid, signal.SIGTERM)
    print('### kill ros_main_thread {} ! ###'.format(ros_main_thread_pid))

