import numpy as np
import os
import sys
import tensorflow as tf
import time

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
import PIL.Image
# get_ipython().run_line_magic('matplotlib', 'inline')


# ## Object detection imports
# Here are the imports from the object detection module.
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

from glob import glob

# ros related
## #!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from zzz_perception_msgs.msg import TrafficLightDetection, TrafficLightDetectionArray

USE_ROS = True #False #True
ROS_PUB = True

# ## Model preparation
def cvtImageToNumpyArray(image): # image is PIL Image object
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape(
        (im_height, im_width, 3)).astype(np.uint8)

def loadLabelMap(path_to_labls, num_classes):
    label_map = label_map_util.load_labelmap(path_to_labls)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    print(category_index)
    return category_index

def loadGraph(model):
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(model, 'rb') as fid: #ssd_inception_real_model
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

    return detection_graph

def loadImagesPath(image_dir):
    print(os.path.join(image_dir, '*.jpg'))
    IMAGE_PATHS = glob(os.path.join(image_dir, '*.jpg'))
    print("Length of test images:", len(IMAGE_PATHS))
    return IMAGE_PATHS

class TrafLightDetc:
    def __init__(self):
        self.initNetwork()

    def initNetwork(self):
        ssd_inception_sim_model = 'frozen_models/frozen_sim_inception/frozen_inference_graph.pb'
        ssd_inception_real_model = 'frozen_models/frozen_real_inception/frozen_inference_graph.pb'
        faster_rcnn_sim_model = 'frozen_models/faster_rcnn_frozen_sim/frozen_inference_graph.pb'
        faster_rcnn_real_model = 'frozen_models/faster_rcnn_frozen_real/frozen_inference_graph.pb'

        PATH_TO_LABELS = 'label_map.pbtxt'
        NUM_CLASSES = 14

        # ## Loading label map
        # Label maps map indices to category names, so that when our convolution network predicts 2, we know that this corresponds to Red. 
        # Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine.
        self.category_index = loadLabelMap(PATH_TO_LABELS, NUM_CLASSES)

        # ## Detection
        # ## 1. Testing SSD Inception Models
        # ### 1.1 Testing model trained on simulator on simulator images
        detection_graph = loadGraph(ssd_inception_sim_model)

        PATH_TO_TEST_IMAGES_DIR = 'test_images_sim' #PATH_TO_TEST_IMAGES_DIR = 'test_images_udacity'
        self.TEST_IMAGE_PATHS = loadImagesPath(PATH_TO_TEST_IMAGES_DIR)

        # Size, in inches, of the output images.
        self.IMAGE_SIZE = (12, 8)

        ## initialize the tensorflow session
        self.gf = detection_graph.as_default()
        self.gf.__enter__()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(config=config, graph=detection_graph)

        ## define wanted tensors in the graph
        # Definite input and output Tensors for detection_graph
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        
    def __del__(self):
        self.sess.close()
        self.gf.__exit__(None, None, None)

    def run(self):
        if USE_ROS:
            self.runRos()
        else:
            self.runImgs()

    def runRos(self):
        rospy.init_node('traf_light_detc') # , anonymous=True
        self.bridge = CvBridge()
        # self.map_state_in = None
        # self.cur_lane = 0
        # self.total_lane = 3
        rospy.Subscriber("/carla/ego_vehicle/camera/rgb/Telephoto/image_color", Image, self.callbackImg)
        if ROS_PUB:
            # rospy.Subscriber("lane_num", MapState, self.callbackLane)
            self.pub = rospy.Publisher("/carla/traffic_lights_detection", TrafficLightDetectionArray, queue_size=10)
        rospy.spin()
    
    def runImgs(self):
        for image_path in self.TEST_IMAGE_PATHS:
            image = PIL.Image.open(image_path)
            image_np = cvtImageToNumpyArray(image)
            self.inferenceImg(image_np)

    # message type is sensor_msgs.msg.Image
    def callbackImg(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        self.inferenceImg(cv_image)

    # # message type is MapState
    # def callbackLane(self, data): 
    #     self.map_state_in = data
    #     self.total_lane = len(data.lanes)
    #     self.cur_lane = data.ego_lane_index

    def inferenceImg(self, image_np):
        
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        time0 = time.time()

        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})

        time1 = time.time()

        boxes = np.squeeze(boxes) # ymin, xmin, ymax, xmax = box according to visualization_utils.py
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        
        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np, boxes, classes, scores,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=6)
        
        # plt.figure(figsize= self.IMAGE_SIZE)
        # plt.imshow(image_np)
        # plt.show()
        cv2.imshow("traffic_light output", image_np[:,:,::-1])
        cv2.waitKey(1)

        self.min_score_thresh = .50
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > self.min_score_thresh:

                class_name = self.category_index[classes[i]]['name']
                print('{}'.format(class_name), scores[i])
                
                # fx =  0.97428
                # fy =  1.73205
                # perceived_width_x = (boxes[i][3] - boxes[i][1]) * 800
                # perceived_width_y = (boxes[i][2] - boxes[i][0]) * 600

                # ymin, xmin, ymax, xmax = box
                # depth_prime = (width_real * focal) / perceived_width
                # perceived_depth_x = ((.1 * fx) / perceived_width_x)
                # perceived_depth_y = ((.3 * fy) / perceived_width_y )

                # estimated_distance = round((perceived_depth_x + perceived_depth_y) / 2)
                # print("Distance (metres)", estimated_distance)
                print("Time in milliseconds", (time1 - time0) * 1000, "\n")
                
        if USE_ROS and ROS_PUB:
            # self.rosSend(classes, boxes)
            self.rosSendLightsOnly(classes, boxes, scores)
            
    def rosSendLightsOnly(self, classes, boxes, scores):
        boxes_x_list = boxes[:, 2].tolist()
        boxes_x_sorted_idx = sorted(range(len(boxes_x_list)),key=boxes_x_list.__getitem__, reverse=True) 
        classes_list = classes.tolist()
        classes_list = [classes_list[i] for i in boxes_x_sorted_idx]
        scores_list = scores.tolist()
        scores_list = [scores_list[i] for i in boxes_x_sorted_idx]

        light_msg = TrafficLightDetectionArray()
        for i in range(boxes.shape[0]):
            if scores is None or scores_list[i] > self.min_score_thresh:
                light_single = TrafficLightDetection()
                if self.category_index[classes_list[i]]['name'] == 'Green':
                    light_single.traffic_light_state = TrafficLightDetection.TRAFFIC_LIGHT_GREEN
                elif self.category_index[classes_list[i]]['name'] == 'Red':
                    light_single.traffic_light_state = TrafficLightDetection.TRAFFIC_LIGHT_RED
                elif self.category_index[classes_list[i]]['name'] == 'Yellow':
                    light_single.traffic_light_state = TrafficLightDetection.TRAFFIC_LIGHT_YELLOW
                
                light_msg.detections.append(light_single)
        
        self.pub.publish(light_msg)


    def rosSend(self, classes, boxes):
        classes_list = classes.tolist()
        boxes_x_list = boxes[:, 2].tolist()
        classes_list = sorted(classes_list, key=boxes_x_list, reverse=True)

        temp_map_state = self.map_state_in
        temp_cur_lane = self.cur_lane
        temp_total_lane = self.total_lane
        if boxes.shape[0] == 1:
            for i in range(temp_total_lane):
                if self.category_index[classes_list[0]]['name'] == 'Red':
                    temp_map_state.lanes[i].traffic_light_state = 1
                elif self.category_index[classes_list[0]]['name'] == 'Yellow':
                    temp_map_state.lanes[i].traffic_light_state = 2
                elif self.category_index[classes_list[0]]['name'] == 'Green':
                    temp_map_state.lanes[i].traffic_light_state = 3
        elif boxes.shape[0] > 1 and boxes.shape[0] == temp_total_lane:
            for i in range(temp_total_lane):
                if self.category_index[classes_list[i]]['name'] == 'Red':
                    temp_map_state.lanes[i].traffic_light_state = 1
                elif self.category_index[classes_list[i]]['name'] == 'Yellow':
                    temp_map_state.lanes[i].traffic_light_state = 2
                elif self.category_index[classes_list[i]]['name'] == 'Green':
                    temp_map_state.lanes[i].traffic_light_state = 3
        elif boxes.shape[0] > 1 and boxes.shape[0] != temp_total_lane:
            red = True
            for i in range(boxes.shape[0]):
                if self.category_index[classes_list[i]]['name'] == 'Green':
                    red = False
            for i in range(temp_total_lane):
                if red:
                    temp_map_state.lanes[i].traffic_light_state = 1
                else:
                    temp_map_state.lanes[i].traffic_light_state = 3

        self.pub.publish(temp_map_state)

        # self.stop = False
        # print("Current lane", temp_cur_lane)
        # if boxes.shape[0] == 1 and self.category_index[classes_list[0]]['name'] == 'Red':
        #     self.stop = True
        # elif boxes.shape[0] > 1:
        #     if boxes.shape[0] == temp_total_lane:
        #         if self.category_index[classes_list[temp_cur_lane]]['name'] == 'Red':
        #             self.stop = True
        #     else:
        #         self.stop = True
        #         for i in range(boxes.shape[0]):
        #             if self.category_index[classes_list[i]]['name'] == 'Green':
        #                 self.stop = False




if __name__ == '__main__':
    traf = TrafLightDetc()
    traf.run()