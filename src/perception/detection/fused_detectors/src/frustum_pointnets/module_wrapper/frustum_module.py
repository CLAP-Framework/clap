import importlib

from ..train import provider
from ..kitti.kitti_object import get_lidar_in_image_fov
from ..models.model_util import g_type2class, g_class2type, g_type2onehotclass

import tensorflow as tf
import numpy as np

# def init_args():
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--gpu', type=int, default=0, help='GPU to use [default: GPU 0]')
#     parser.add_argument('--num_point', type=int, default=1024, help='Point Number [default: 1024]')
#     parser.add_argument('--model', default='frustum_pointnets_v1', help='Model name [default: frustum_pointnets_v1]')
#     parser.add_argument('--model_path', default='log/model.ckpt', help='model checkpoint file path [default: log/model.ckpt]')
#     parser.add_argument('--batch_size', type=int, default=32, help='batch size for inference [default: 32]')
#     parser.add_argument('--output', default='test_results', help='output file/folder name [default: test_results]')
#     parser.add_argument('--data_path', default=None, help='frustum dataset pickle filepath [default: None]')
#     parser.add_argument('--from_rgb_detection', action='store_true', help='test from dataset files from rgb detection.')
#     parser.add_argument('--idx_path', default=None, help='filename of txt where each line is a data idx, used for rgb detection -- write <id>.txt for all frames. [default: None]')
#     parser.add_argument('--dump_result', action='store_true', help='If true, also dump results to .pickle file')
#     FLAGS = parser.parse_args()

#     return FLAGS


def batch_inference_network(pcs, one_hot_vecs, sess, ops):
    # compared with original inference function, this function works for each batch
    # and does not return scores

    ep = ops['end_points'] 
    feed_dict = {\
        ops['pointclouds_pl']: pcs,
        ops['one_hot_vec_pl']: one_hot_vecs,
        ops['is_training_pl']: False}

    batch_logits, batch_centers, \
    batch_heading_scores, batch_heading_residuals, \
    batch_size_scores, batch_size_residuals = \
        sess.run([ops['logits'], ops['center'],
            ep['heading_scores'], ep['heading_residuals'],
            ep['size_scores'], ep['size_residuals']],
            feed_dict=feed_dict)

    batch_outputs = np.argmax(batch_logits, 2)
    batch_heading_cls = np.argmax(batch_heading_scores, 1) # B
    batch_heading_res = np.array([batch_heading_residuals[i,batch_heading_cls[i]] for i in range(len(pcs))])
    batch_size_cls = np.argmax(batch_size_scores, 1) # B
    batch_size_res = np.vstack([batch_size_residuals[i,batch_size_cls[i],:] for i in range(len(pcs))])
    
    return batch_outputs, batch_centers, batch_heading_cls, batch_heading_res, \
    batch_size_cls, batch_size_res

def process_2d_detections(boxes_2d, labels, calib, pc_velo, img, 
                            type_whitelist=['Car', 'Pedestrian', 'Cyclist'], img_height_threshold=25, lidar_point_threshold=5):
    # from extract_frustum_data_rgb_detection in prepare_data.py
    type_list = []
    input_list = []
    frustum_angle_list = []

    for i in range(len(boxes_2d)):
        label = labels[i]
        box_2d = boxes_2d[i]

        if label not in type_whitelist: continue

        pc_in_box_fov, frustum_angle, y_diff = get_pc_in_box_and_angle(calib, pc_velo, img, box_2d )
        
        # Pass objects that are too small
        if y_diff < img_height_threshold or \
            len(pc_in_box_fov)<lidar_point_threshold:
            continue
       
        type_list.append(label )
        input_list.append(pc_in_box_fov )
        frustum_angle_list.append(frustum_angle)

    return type_list, input_list, frustum_angle_list

def from_2d_detections_to_frustum(type_list, input_list, frustum_angle_list, npoints, nchannels, batch_size):
    # from FrustumDataset in provider.py and get_batch_from_rgb_detection in train_utils.py
    point_set_list = []
    rot_angle_list = []
    one_hot_vec_list = []
    for i in range(len(frustum_angle_list)):
        rot_angle = np.pi/2.0 + frustum_angle_list[i]

        # one_hot = True
        cls_type = type_list[i]
        assert(cls_type in ['Car', 'Pedestrian', 'Cyclist'])
        one_hot_vec = np.zeros((3))
        one_hot_vec[g_type2onehotclass[cls_type]] = 1

        # rotate_to_center = True
        point_set = np.copy(input_list[i])
        point_set = provider.rotate_pc_along_y(point_set, rot_angle)
        # Resample
        choice = np.random.choice(point_set.shape[0], size=npoints, replace=True)
        point_set = point_set[choice, 0:nchannels]

        point_set_list.append(point_set)
        rot_angle_list.append(rot_angle)
        one_hot_vec_list.append(one_hot_vec)
    
    while len(point_set_list) < batch_size:
        point_set_list.append(point_set)
        rot_angle_list.append(rot_angle)
        one_hot_vec_list.append(one_hot_vec)

    if len(point_set_list) > batch_size:
        point_set_list = point_set_list[0:batch_size]
        rot_angle_list = rot_angle_list[0:batch_size]
        one_hot_vec_list = one_hot_vec_list[0:batch_size]

    return point_set_list, rot_angle_list, one_hot_vec_list

def get_pc_in_box_and_angle(calib, pc_velo, img, box_2d ):
    pc_rect = np.zeros_like(pc_velo)
    pc_rect[:,0:3] = calib.project_velo_to_rect(pc_velo[:,0:3])
    pc_rect[:,3] = pc_velo[:,3]
    
    img_height, img_width, img_channel = img.shape
    _, pc_image_coord, img_fov_inds = get_lidar_in_image_fov(\
        pc_velo[:,0:3], calib, 0, 0, img_width, img_height, True)

    # 2D BOX: Get pts rect backprojected 
    xmin,ymin,xmax,ymax = box_2d
    box_fov_inds = (pc_image_coord[:,0]<xmax) & \
        (pc_image_coord[:,0]>=xmin) & \
        (pc_image_coord[:,1]<ymax) & \
        (pc_image_coord[:,1]>=ymin)
    box_fov_inds = box_fov_inds & img_fov_inds
    pc_in_box_fov = pc_rect[box_fov_inds,:]
    
    # Get frustum angle (according to center pixel in 2D BOX)
    box2d_center = np.array([(xmin+xmax)/2.0, (ymin+ymax)/2.0])
    uvdepth = np.zeros((1,3))
    uvdepth[0,0:2] = box2d_center
    uvdepth[0,2] = 20 # some random depth
    box2d_center_rect = calib.project_image_to_rect(uvdepth)
    frustum_angle = -1 * np.arctan2(box2d_center_rect[0,2],
        box2d_center_rect[0,0])    

    return pc_in_box_fov, frustum_angle, ymax-ymin


class FrustumModule(object):
    def __init__(self, params):
        # ros subscriber: point cloud, 2D boxes (position and label), camera intrinsics, cam-lidar extrinsics
        # ros publisher: 3D boxes(position and label)
        # initialize parameters
        # initialize the network instance
        # if params is None:
        #     params = init_args()
        self.FLAGS = params
        # Set training configurations
        self.BATCH_SIZE = self.FLAGS.batch_size
        self.MODEL_PATH = self.FLAGS.model_path
        self.GPU_INDEX = self.FLAGS.gpu
        self.NUM_POINT = self.FLAGS.num_point
        self.MODEL = importlib.import_module(self.FLAGS.model)
        self.NUM_CLASSES = 2
        self.NUM_CHANNEL = 4

        self.sess, self.ops = self.get_session_and_ops(batch_size=self.BATCH_SIZE, num_point=self.NUM_POINT)
        
    
    def get_session_and_ops(self, batch_size, num_point):
        ''' Define model graph, load model parameters,
        create session and return session handle and tensors
        '''
        with tf.Graph().as_default():
            with tf.device('/gpu:'+str(self.GPU_INDEX)):
                pointclouds_pl, one_hot_vec_pl, labels_pl, centers_pl, \
                heading_class_label_pl, heading_residual_label_pl, \
                size_class_label_pl, size_residual_label_pl = \
                    self.MODEL.placeholder_inputs(batch_size, num_point)
                is_training_pl = tf.placeholder(tf.bool, shape=())
                end_points = self.MODEL.get_model(pointclouds_pl, one_hot_vec_pl,
                    is_training_pl)
                loss = self.MODEL.get_loss(labels_pl, centers_pl,
                    heading_class_label_pl, heading_residual_label_pl,
                    size_class_label_pl, size_residual_label_pl, end_points)
                saver = tf.train.Saver()

            # Create a session
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            config.allow_soft_placement = True
            sess = tf.Session(config=config)

            # Restore variables from disk.
            saver.restore(sess, self.MODEL_PATH)
            ops = {'pointclouds_pl': pointclouds_pl,
                'one_hot_vec_pl': one_hot_vec_pl,
                'labels_pl': labels_pl,
                'centers_pl': centers_pl,
                'heading_class_label_pl': heading_class_label_pl,
                'heading_residual_label_pl': heading_residual_label_pl,
                'size_class_label_pl': size_class_label_pl,
                'size_residual_label_pl': size_residual_label_pl,
                'is_training_pl': is_training_pl,
                'logits': end_points['mask_logits'],
                'center': end_points['center'],
                'end_points': end_points,
                'loss': loss}
            return sess, ops

    def run(self, pcs_in, img, labels, boxes_2d, calib_cam_imu):
        """

        """
        hs = []
        ws = []
        ls = []
        txs = []
        tys = []
        tzs = []
        rys = []
        type_list = []
        if len(boxes_2d) != 0: 
            type_list, input_list, frustum_angle_list = \
                process_2d_detections(boxes_2d, labels, calib_cam_imu, pcs_in, img)

            if len(input_list) != 0: 
                true_length = len(input_list)

                hs = [None]*true_length
                ws = [None]*true_length
                ls = [None]*true_length
                txs = [None]*true_length
                tys = [None]*true_length
                tzs = [None]*true_length
                rys = [None]*true_length
                        

                point_set_list, rot_angle_list, one_hot_vec_list = \
                    from_2d_detections_to_frustum(type_list, input_list, frustum_angle_list, npoints=self.NUM_POINT, nchannels=self.NUM_CHANNEL, batch_size=self.BATCH_SIZE)

                batch_outputs, batch_centers, batch_heading_cls, batch_heading_res, \
                    batch_size_cls, batch_size_res = \
                        batch_inference_network(point_set_list, one_hot_vec_list, self.sess, self.ops)
                
                for i in range( true_length ):
                    hs[i],ws[i],ls[i],txs[i],tys[i],tzs[i],rys[i] = provider.from_prediction_to_label_format(batch_centers[i],
                        batch_heading_cls[i], batch_heading_res[i],
                        batch_size_cls[i], batch_size_res[i], rot_angle_list[i])

        return hs, ws, ls, txs, tys, tzs, rys, type_list
            

	



