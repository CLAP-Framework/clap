'''
Inverse Perspective Mapping
Credit to `write_ipm_yml.py` by minghan
'''

import os
import cv2
import pcl
import numpy as np
import scipy.spatial.transform as sst

import rospy
import tf
import tf2_ros as tf2
from zzz_perception_msgs.msg import DetectionBox2DArray, DetectionBox, DetectionBoxArray
from zzz_common.params import IntrinsicListener

class InversePerspectiveMapping():
    '''
    This class provide utilities to project coordinate using Inverse Perspective Mapping (IPM)
    '''

    def __init__(self, height, rotation=None):
        '''
        :param height: camera height (to a perfect ground)
        '''
        if rotation == None:
            self.R = sst.Rotation([0,0,0,1])
        elif type(rotation) != sst.Rotation:
            raise ValueError("You should input a scipy.spatial.transform.Rotation object as rotation")
        else:
            self.R = rotation
        self.R = self.R.as_dcm()
        self.h = height

        self._intrinsic_assigned = False
        self._bevparams_assigned = False

    def setIntrinsics(self, cx, cy, fx=None, fy=None, fov=None):
        '''
        Generate intrinsic matrix from camera params.

        :param cx: camera intrinsics
        :param cy: camera intrinsics
        :param fx: camera intrinsics
        :param fy: camera intrinsics
        :param fov: camera field of view. If it's not provided, then we try to calculate from fx, fy
        :param rotation: camera rotation. It's zero when camera x is pointing forward, y pointing leftward and z pointing downward.
        '''
        self.M_in = np.zeros((3,3))
        if fov is None:
            if fx is None and fy is None:
                fx = cx
                fy = fx
        elif fx is None and fy is None:
            fx = cx / np.tan(fov/2.0/180.0*np.pi)
            fy = fx
        else:
            raise ValueError('Cannot give fx/fy and fov at the same time')

        # Here we assume the axis is x-front, y-right, z-down
        self.M_in[0, 0] = cx - 0.5
        self.M_in[0, 1] = fx
        self.M_in[1, 0] = cy - 0.5
        self.M_in[1, 2] = fy
        self.M_in[2, 0] = 1

        self._intrinsic_assigned = True

    def setBEV(self, x_end, y_width, u_bev=0, v_bev=0, x_start=0, y_offset=0):
        '''
        Generate Bird's Eye View related matrices, set params for BEV image generation
        
        :param x_end: BEV area farthest distance
        :param x_start: BEV area closest distance
        :param y_width: BEV area lateral span
        :param y_offset: BEV area lateral offset. 0 means that the BEV area is align with x axis
        :param u_bev: BEV Image width
        :param v_bev: BEV Image height
        '''
        if u_bev == 0 and v_bev == 0:
            u_bev = y_width
            v_bev = x_end - x_start
        self.u_bev_width = u_bev
        self.v_bev_height = v_bev
        self.x_bev_long_dist = float(x_end - x_start)
        self.x_bev_long_start = float(x_start)
        self.x_bev_long_end = float(x_end)
        self.y_bev_lat_dist = float(y_width)
        self.y_bev_lat_start = y_offset - float(y_width)/2
        self.y_bev_lat_end = y_offset + float(y_width)/2

        self.M_bev2world = self._calc_bev2world()
        self.M_world2bev = self._calc_world2bev()
        self.M_img2bev = self._calc_img2bev()
        self.M_bev2img = self._calc_bev2img()

        self._bevparams_assigned = True

    def bev2world(self, us, vs):
        '''
        Convert BEV image coordinate to world coordinate
        '''
        assert us.shape == vs.shape, "u and v of of different shape"

        ## Alternative way
        # ys = (us+0.5) /self.u_bev_width * self.y_bev_lat_dist + self.y_bev_lat_start
        # xs = (self.v_bev_height - (vs+0.5) ) / self.v_bev_height * self.x_bev_long_dist + self.x_bev_long_start

        ts = np.ones_like(us)
        coord_bev = np.vstack((us, vs, ts))
        coord_world = self.M_bev2world.dot(coord_bev)
        xs = coord_world[0, :]
        ys = coord_world[1, :]
        return xs, ys
    
    def world2bev(self, xs, ys):
        '''
        Convert world coordinate to BEV image coordinate
        '''
        assert xs.shape == ys.shape, "x and y of different shape"

        ## Alternative way
        # us = (ys - self.y_bev_lat_start) / self.y_bev_lat_dist * self.u_bev_width - 0.5
        # vs = self.v_bev_height - (xs - self.x_bev_long_start) / self.x_bev_long_dist * self.v_bev_height - 0.5

        zs = np.ones_like(xs) * self.h
        coord_world = np.vstack((xs, ys, zs))
        coord_bev = self.M_world2bev.dot(coord_world)
        us = coord_bev[0, :]
        vs = coord_bev[1, :]
        return us, vs

    def world2img(self, xs, ys):
        '''
        Convert world coordinate to normal image coordinate
        '''
        assert xs.shape == ys.shape, "x and y of different shape"

        zs = np.ones_like(xs)*self.h
        coord_world = np.vstack((xs, ys, zs))
        coord_img = self.M_in.dot(self.R).dot(coord_world)
        us = coord_img[0, :] / coord_img[2, :]
        vs = coord_img[1, :] / coord_img[2, :]
        return us, vs
        
    def img2world(self, us, vs):
        '''
        Convert normal image coordinate to world coordinate
        '''
        assert us.shape == vs.shape, "u and v of of different shape"
        ts = np.ones_like(us)
        coord_img = np.vstack((us, vs, ts))
        mat = np.linalg.inv(self.M_in.dot(self.R))
        coord_world_psudo = mat.dot(coord_img)
        z = coord_world_psudo[2, :]
        coord_world_psudo = coord_world_psudo / z * self.h
        xs = coord_world_psudo[0, :]
        ys = coord_world_psudo[1, :]
        return xs, ys

    def bev2img(self, u_bev, v_bev):
        '''
        Convert BEV image coordinate to normal image coordinate
        '''
        ## Alternative way
        # x_world, y_world = self.bev2world(u_bev, v_bev)
        # u_img, v_img = self.world2img(x_world, y_world)

        t_bev = np.ones_like(u_bev)
        coord_bev = np.vstack((u_bev, v_bev, t_bev))
        coord_img = self.M_bev2img.dot(coord_bev)
        u_img = coord_img[0, :] / coord_img[2, :]
        v_img = coord_img[1, :] / coord_img[2, :]
        
        return u_img, v_img
    
    def img2bev(self, *kargs):
        '''
        Convert normal image coordinate to BEV image coordinate

        :params kargs: The input argument could be either (u_img, v_img) or directly image
        '''
        if len(kargs) == 2:
            ## Alternative way
            # x_world, y_world = self.img2world(u_img, v_img)
            # u_bev, v_bev = self.world2bev(x_world, y_world)

            u_img, v_img = kargs
            t_img = np.ones_like(u_img)
            coord_img = np.vstack((u_img, v_img, t_img))
            coord_bev = self.M_img2bev.dot(coord_img)
            u_bev = coord_bev[0, :] / coord_bev[2, :]
            v_bev = coord_bev[1, :] / coord_bev[2, :]
            return u_bev, v_bev

        elif len(kargs) == 1:
            return self._getBevFromImg_method3(kargs[0])

        else:
            raise ValueError("Invalid number of input arguments!")

    def _calc_img2bev(self):
        '''
        Get perspective transform from normal image to BEV image
        '''

        mat = np.linalg.inv(self.M_in.dot(self.R).dot(self.M_bev2world))
        mat = mat/mat[2,2]
        return mat

    def _calc_bev2img(self):
        '''
        Get perspective transform from BEV image to normal image
        '''

        mat = self.M_in.dot(self.R).dot(self.M_bev2world)
        mat = mat/mat[2,2]
        return mat

    def _calc_bev2world(self):
        '''
        Calculate the matrix that convert BEV coordinates to world
        '''

        mat = np.zeros((3,3))
        mat[0, 1] = -self.x_bev_long_dist / self.v_bev_height
        mat[0, 2] = self.x_bev_long_dist + self.x_bev_long_start - 0.5*self.x_bev_long_dist / self.v_bev_height
        mat[1, 0] = self.y_bev_lat_dist / self.u_bev_width
        mat[1, 2] = self.y_bev_lat_start + 0.5*self.y_bev_lat_dist / self.u_bev_width
        mat[2, 2] = self.h
        return mat

    def _calc_world2bev(self):
        '''
        Calculate the matrix that convert world coordinates to BEV

        Note XXX: Actually M_world2bev is equivalent to the inverse of M_bev2world
        '''
        mat = np.zeros((3,3))
        mat[0, 1] = self.u_bev_width / self.y_bev_lat_dist
        mat[0, 2] = (-self.u_bev_width / self.y_bev_lat_dist * self.y_bev_lat_start - 0.5) / self.h
        mat[1, 0] = -self.v_bev_height / self.x_bev_long_dist
        mat[1, 2] = (self.v_bev_height / self.x_bev_long_dist * self.x_bev_long_start + self.v_bev_height - 0.5) / self.h
        mat[2, 2] = 1 / self.h
        return mat

    def _getBevFromImg_method1(self, img):
        u_bev = np.arange(self.u_bev_width)
        v_bev = np.arange(self.v_bev_height)

        u_bev_grid, v_bev_grid = np.meshgrid(u_bev, v_bev)
        u_bev_flat = u_bev_grid.flatten()
        v_bev_flat = v_bev_grid.flatten()

        u_img_flat, v_img_flat = self.bev2img(u_bev_flat, v_bev_flat)

        u_img = u_img_flat.reshape((self.v_bev_height, self.u_bev_width)).astype(np.float32)
        v_img = v_img_flat.reshape((self.v_bev_height, self.u_bev_width)).astype(np.float32)

        bev = cv2.remap(img, u_img, v_img, cv2.INTER_LINEAR)

        return bev, u_img, v_img

    def _getBevFromImg_method2(self, img):
        x_world = np.array([5, 50, 50, 5])
        y_world = np.array([-3, -3, 3, 3])
        u_img, v_img = self.world2img(x_world, y_world)
        u_bev, v_bev = self.world2bev(x_world, y_world)

        pts_img = np.vstack((u_img, v_img)).transpose((1, 0)).astype(np.float32)
        pts_bev = np.vstack((u_bev, v_bev)).transpose((1, 0)).astype(np.float32)

        perspective_transform = cv2.getPerspectiveTransform(pts_img, pts_bev)
        bev = cv2.warpPerspective(img, perspective_transform, (self.u_bev_width, self.v_bev_height))
        return bev, pts_img, pts_bev

    def _getBevFromImg_method3(self, img):
        perspective_transform = self.M_img2bev
        bev = cv2.warpPerspective(img, perspective_transform, (self.u_bev_width, self.v_bev_height))
        return bev

class LidarExtrinsicProjection:
    '''
    This class project the points from lidar into image and then extract the lidar cluster
    from image based on criteria.
    '''
    def __init__(self, criterion="closest", params=None):
        '''
        :param criterion: the strategy to choose cluster. `closest`: select closest points and cluster based on mutual distance threshold.
        :type criterion: str
        :param params: parameters for the selected criterion
        :type params: tuple
        '''
        self._criterion = criterion
        self._params = params
        if not self._params: # default params
            if criterion == "closest":
                self._params = (1, 1) # mutual distance threshold, discard point threshold at distance

        self._tfbuffer = tf2.Buffer()
        self._tflistener = tf2.TransformListener(self._tfbuffer)
        self._intri_buffer = IntrinsicListener()
        self._cloud_data = None # transformed cloud array
        self._cloud_frame = None # point cloud frame
        
    def receive_point_cloud(self, cloud_msg):
        self._cloud_frame = cloud_msg.header.frame_id
        self._cloud_data = pcl.PointCloud(cloud_msg)

        self._tfcloud = pcl.PointCloud(cloud_msg)

    def receive_detections(self, detections):
        if self._criterion == "closest":
            return self._process_from_closest(detections)
        return None

    def _process_from_closest(self, detections):
        # Fetch parameters
        mutual_dist_thres, close_zthres = self._params

        # Fetch extrinsics
        if not self._cloud_frame:
            return None # not yet initialized

        assert type(detections) == DetectionBox2DArray
        image_frame = detections.header.frame_id

        try:
            rt = self._tfbuffer.lookup_transform(self._cloud_frame, image_frame, detections.header.stamp)
            intri = self._intri_buffer.lookupCameraInfo(image_frame)
        except tf2.LookupException as e:
            rospy.logwarn("TF lookup error: %s", str(e))
            return None

        # Project lidar to cam frame
        R = tf.transformations.quaternion_matrix([rt.transform.rotation.x, rt.transform.rotation.y, rt.transform.rotation.z, rt.transform.rotation.w])[:3,:3]
        T = np.array([rt.transform.translation.x, rt.transform.translation.y, rt.transform.translation.z]).reshape(3, 1)
        cloud_transformed = R.dot(self._cloud_data.xyz.T) + T

        K = np.reshape(intri.K, (3,3))
        zloc = cloud_transformed[2,:]
        proj = K.dot(cloud_transformed) # FIXME: distortion parameters are currently ignored
        xloc = proj[0,:] / proj[2,:]
        yloc = proj[1,:] / proj[2,:]

        # Calculate points for each detection
        # FIXME: rotation of the detection box are not considered
        dist_list = np.linalg.norm(cloud_transformed, axis=0)
        result = DetectionBoxArray()
        result.header.frame_id = self._cloud_frame
        result.header.stamp = detections.header.stamp
        for det in detections.detections:
            mask_x = np.abs(xloc - det.bbox.pose.x) <= det.bbox.dimension.length_x/2
            mask_y = np.abs(yloc - det.bbox.pose.y) <= det.bbox.dimension.length_y/2
            mask = mask_x & mask_y & (zloc > close_zthres)
            
            closest_index = np.argmin(dist_list[mask])
            valid_indices = np.abs(dist_list[mask] - dist_list[mask][closest_index]) < mutual_dist_thres
            valid_points = self._cloud_data.xyz[mask][valid_indices]

            trans_det = DetectionBox()
            trans_det.source_cloud = pcl.create_xyz(valid_points).to_msg()
            trans_det.source_img = det.source_img
            trans_det.source_frame = image_frame
            center_x, center_y, center_z = np.mean(valid_points, axis=0)
            trans_det.bbox.pose.pose.position.x = center_x
            trans_det.bbox.pose.pose.position.y = center_y
            trans_det.bbox.pose.pose.position.z = center_z

            result.detections.append(trans_det)

        return result
