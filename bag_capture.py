#!/usr/bin/python
import time
import roslib
import rospy
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridgeError, CvBridge
from queue import Queue
import argparse
import threading
import copy

from zzz_driver_msgs.msg import RigidBodyStateStamped
from zzz_perception_msgs.msg import TrackingBox, TrackingBoxArray 
from xpmotors_can_msgs.msg import AutoStateEx

roslib.load_manifest('rosbag')

# parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
# parser.add_argument('bagfile', nargs=1, help='input bag file')
# parser.add_argument('--window_seconds', nargs=1, help='max time offset (sec) to correct.', default='120', type=int)
# args = parser.parse_args()

auto_topic = '/xp/auto_state_ex'
left_cam_topic = '/left_usb_cam/image_raw/compressed'
ego_pose_topic = '/zzz/navigation/ego_pose'
obs_topic  = '/zzz/perception/objects_tracked'

# window_seconds = args.window_seconds
window_seconds = 10
pose_queue  = Queue(100 * window_seconds)
obs_queue   = Queue(10 * window_seconds)
image_queue = Queue(10 * window_seconds) 

bag = None

def write2bag(pose_queue, obs_queue, image_queue):

    filename = time.strftime("%Y-%m-%d_%H:%M:%S.bag", time.localtime()) 
    
    bag = rosbag.Bag(filename, "w")
    for msg in list(pose_queue.queue):
        bag.write(ego_pose_topic, msg)

    for msg in list(obs_queue.queue):
        bag.write(obs_topic, msg)

    for msg in list(image_queue.queue):
        bag.write(left_cam_topic, msg)
    
    bag.close()
    print('*** write {} done! ***'.format(filename))

    

# 2-autopilot, 0-manually
last_autostate = 0

def autostate_callback(msg):
    
    global last_autostate
    global pose_queue
    global obs_queue
    global image_queue

    print('*** autostate {}, last {}'.format(msg.CurDriveMode, last_autostate))
    global last_autostate
    if msg.CurDriveMode == 0 and last_autostate == 2:
        rospy.loginfo("*** launch thread record bag file ###")
        t = threading.Thread(
            target = write2bag, 
            args = (pose_queue, obs_queue, image_queue))
        t.setDaemon(True)
        t.start()

    last_autostate = msg.CurDriveMode



def ego_pose_callback(msg):
    if not pose_queue.full():
        pose_queue.put(msg)
    else:
        pose_queue.get()
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


def main():

    rospy.init_node("bug-catcher", anonymous=True)

    try:    
        global auto_topic
        global ego_pose_topic
        global obs_topic
        global left_cam_topic
        # print('*** ', auto_topic)

        rospy.Subscriber(auto_topic, AutoStateEx, autostate_callback)
        rospy.Subscriber(left_cam_topic, CompressedImage, image_callback)
        rospy.Subscriber(ego_pose_topic, RigidBodyStateStamped, ego_pose_callback)
        rospy.Subscriber(obs_topic, TrackingBoxArray, obstacles_callback)

        rospy.loginfo("create sub done, start spin loop...")
        rospy.spin()
    finally:

        rospy.loginfo("*** all done ***")


if __name__ == "__main__":
    main()