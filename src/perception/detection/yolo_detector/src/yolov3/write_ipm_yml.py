import cv2
import numpy as np
import os

def R_yaw(a):
    a = a/180*np.pi
    mat = np.zeros((3,3))
    mat[0,0] = np.cos(a)
    mat[0,1] = -np.sin(a)
    mat[1,0] = np.sin(a)
    mat[1,1] = np.cos(a)
    mat[2,2] = 1
    return mat

def R_pitch(a):
    a = a/180*np.pi
    mat = np.zeros((3,3))
    mat[0,0] = np.cos(a)
    mat[0,2] = np.sin(a)
    mat[1,1] = 1
    mat[2,0] = -np.sin(a)
    mat[2,2] = np.cos(a)
    return mat

def R_roll(a):
    a = a/180*np.pi
    mat = np.zeros((3,3))
    mat[0,0] = 1
    mat[1,1] = np.cos(a)
    mat[1,2] = -np.sin(a)
    mat[2,1] = np.sin(a)
    mat[2,2] = np.cos(a)
    return mat

def R(yaw, pitch, roll):
    mat = R_yaw(yaw).dot(R_pitch(pitch)).dot(R_roll(roll))
    return mat

def M_in(cx, cy, fx=None, fy=None, fov=None):
    mat = np.zeros((3,3))
    if fov is None:
        if fx is None and fy is None:
            fx = cx
            fy = fx
        mat[0, 0] = cx - 0.5
        mat[0, 1] = fx
        mat[1, 0] = cy - 0.5
        mat[1, 2] = fy
        mat[2, 0] = 1
    elif fx is None and fy is None:
        fx = cx / np.tan(fov/2.0/180.0*np.pi)
        fy = fx
        mat[0, 0] = cx - 0.5
        mat[0, 1] = fx
        mat[1, 0] = cy - 0.5
        mat[1, 2] = fy
        mat[2, 0] = 1
    else:
        raise Exception('Cannot give fx/fy and fov at the same time')
    return mat

def M_bev2world(u_bev_width, v_bev_height, x_long_dist, y_lat_dist, x_long_start, y_lat_start, h=1):
    mat = np.zeros((3,3))
    mat[0, 1] = -x_long_dist/v_bev_height
    mat[0, 2] = x_long_dist + x_long_start - 0.5*x_long_dist/v_bev_height
    mat[1, 0] = y_lat_dist/u_bev_width
    mat[1, 2] = y_lat_start + 0.5*y_lat_dist/u_bev_width
    mat[2, 2] = h #1
    return mat

def M_world2bev(u_bev_width, v_bev_height, x_long_dist, y_lat_dist, x_long_start, y_lat_start, h=1):
    mat = np.zeros((3,3))
    mat[0, 1] = u_bev_width/y_lat_dist
    mat[0, 2] = ( -u_bev_width/y_lat_dist*y_lat_start - 0.5 ) / h
    mat[1, 0] = -v_bev_height/x_long_dist
    mat[1, 2] = ( v_bev_height/x_long_dist*x_long_start + v_bev_height - 0.5 ) /h
    mat[2, 2] = 1 / h
    return mat


class CamIntrExtr():
    def __init__(self, yaw, pitch, roll, height):
        self.R = R(yaw, pitch, roll)
        self.h = height

    def setIntr(self, cx, cy, fx=None, fy=None, fov = None):
        self.M_in = M_in(cx, cy, fx, fy, fov)
        self.img_width = 2*cx
        self.img_height = 2*cy

    def setBEV(self, x_end, y_width, u_bev=0, v_bev=0, x_start = 0, y_offset = 0):
        if u_bev == 0 and v_bev == 0:
            u_bev = y_width
            v_bev = x_end - x_start
        self.u_bev_width = u_bev
        self.v_bev_height = v_bev
        self.x_bev_long_dist = x_end - x_start
        self.x_bev_long_start = x_start
        self.x_bev_long_end = x_end
        self.y_bev_lat_dist = y_width
        self.y_bev_lat_start = y_offset - y_width/2
        self.y_bev_lat_end = y_offset + y_width/2

        self.M_bev2world = M_bev2world(self.u_bev_width, self.v_bev_height, 
        self.x_bev_long_dist, self.y_bev_lat_dist, self.x_bev_long_start, self.y_bev_lat_start, self.h)

        self.M_world2bev = M_world2bev(self.u_bev_width, self.v_bev_height, 
        self.x_bev_long_dist, self.y_bev_lat_dist, self.x_bev_long_start, self.y_bev_lat_start, self.h)

        self.M_img2bev = self.getPerspectiveTransform_img2bev()
        self.M_bev2img = self.getPerspectiveTransform_bev2img()

    def bev2world(self, us, vs):
        assert us.shape == vs.shape, "u and v of of different shape"
        # ys = (us+0.5) /self.u_bev_width * self.y_bev_lat_dist + self.y_bev_lat_start
        # xs = (self.v_bev_height - (vs+0.5) ) / self.v_bev_height * self.x_bev_long_dist + self.x_bev_long_start
        ts = np.ones_like(us)
        coord_bev = np.vstack((us, vs, ts))
        coord_world = self.M_bev2world.dot(coord_bev)
        xs = coord_world[0, :]
        ys = coord_world[1, :]
        return xs, ys
    
    def world2bev(self, xs, ys):
        assert xs.shape == ys.shape, "x and y of of different shape"
        # us = (ys - self.y_bev_lat_start) / self.y_bev_lat_dist * self.u_bev_width - 0.5
        # vs = self.v_bev_height - (xs - self.x_bev_long_start) / self.x_bev_long_dist * self.v_bev_height - 0.5
        zs = np.ones_like(xs) * self.h
        coord_world = np.vstack((xs, ys, zs))
        ## self.M_world2bev is equivalent to the inverse of self.M_bev2world
        # mat = np.linalg.inv(self.M_bev2world)
        # coord_bev = mat.dot(coord_world)
        coord_bev = self.M_world2bev.dot(coord_world)
        us = coord_bev[0, :]
        vs = coord_bev[1, :]
        return us, vs

    def world2img(self, xs, ys):
        # x front, y right, z down
        assert xs.shape == ys.shape, "x and y of of different shape"
        zs = np.ones_like(xs)*self.h
        coord_world = np.vstack((xs, ys, zs))
        coord_img = self.M_in.dot(self.R).dot(coord_world)
        us = coord_img[0, :] / coord_img[2, :]
        vs = coord_img[1, :] / coord_img[2, :]
        return us, vs
        
    def img2world(self, us, vs):
        assert us.shape == vs.shape, "u and v of of different shape"
        ts = np.ones_like(us)
        coord_img = np.vstack((us, vs, ts))
        mat = np.linalg.inv( self.M_in.dot(self.R) )
        coord_world_psudo = mat.dot( coord_img )
        z = coord_world_psudo[2, :]
        coord_world_psudo = coord_world_psudo / z * self.h
        xs = coord_world_psudo[0, :]
        ys = coord_world_psudo[1, :]
        return xs, ys

    def bev2img(self, u_bev, v_bev):
        # x_world, y_world = self.bev2world(u_bev, v_bev)
        # u_img, v_img = self.world2img(x_world, y_world)
        t_bev = np.ones_like(u_bev)
        coord_bev = np.vstack((u_bev, v_bev, t_bev))
        coord_img = self.M_bev2img.dot(coord_bev)
        u_img = coord_img[0, :] / coord_img[2, :]
        v_img = coord_img[1, :] / coord_img[2, :]
        
        return u_img, v_img
    
    def img2bev(self, u_img, v_img):
        # x_world, y_world = self.img2world(u_img, v_img)
        # u_bev, v_bev = self.world2bev(x_world, y_world)
        t_img = np.ones_like(u_img)
        coord_img = np.vstack((u_img, v_img, t_img))
        coord_bev = self.M_img2bev.dot(coord_img)
        u_bev = coord_bev[0, :] / coord_bev[2, :]
        v_bev = coord_bev[1, :] / coord_bev[2, :]
        
        return u_bev, v_bev

    def getPerspectiveTransform_img2bev(self):
        mat = np.linalg.inv(self.M_in.dot(self.R).dot(self.M_bev2world) )
        mat = mat/mat[2,2]
        return mat

    def getPerspectiveTransform_bev2img(self):
        mat = self.M_in.dot(self.R).dot(self.M_bev2world)
        mat = mat/mat[2,2]
        return mat

    def getBevFromImg_1(self, img):
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

    def getBevFromImg_2(self, img):
        x_world = np.array([5, 50, 50, 5])
        y_world = np.array([-3, -3, 3, 3])
        u_img, v_img = self.world2img(x_world, y_world)
        u_bev, v_bev = self.world2bev(x_world, y_world)

        pts_img = np.vstack((u_img, v_img)).transpose((1, 0)).astype(np.float32)
        pts_bev = np.vstack((u_bev, v_bev)).transpose((1, 0)).astype(np.float32)

        perspective_transform = cv2.getPerspectiveTransform(pts_img, pts_bev)
        print('2', perspective_transform)
        bev = cv2.warpPerspective(img, perspective_transform, (self.u_bev_width, self.v_bev_height))
        return bev, pts_img, pts_bev

    def getBevFromImg_3(self, img):
        perspective_transform = self.M_img2bev
        print('3', perspective_transform)
        bev = cv2.warpPerspective(img, perspective_transform, (self.u_bev_width, self.v_bev_height))
        return bev

if __name__ == "__main__":
    ## camera extrinsics
    yaw = 0
    pitch = 10
    roll = 0
    height = 2.2

    cam = CamIntrExtr(yaw, pitch, roll, height)

    ## camera intrinsics
    img_width = 600
    img_height = 300
    cx = img_width/2
    cy = img_height/2
    fov = 70

    cam.setIntr(cx, cy, fov = fov)

    ## bev setting
    bev_width = 270
    bev_height = 700
    x_end = 80
    y_width = 30

    align_img_and_bev_bot = True

    if align_img_and_bev_bot:
        x_bot, _ = cam.img2world(np.array([0]), np.array([img_height-1]) )
        cam.setBEV(x_end, y_width, bev_width, bev_height, x_bot[0])
    else:
        cam.setBEV(x_end, y_width, bev_width, bev_height)

    # home = os.path.expanduser('~')
    # img_path = os.path.join(home, 'Carla/scenario_runner_cz/image_npy/imgs/cam_img084584.png')
    img_path = "data/cam_img084584.png"
    img = cv2.imread(img_path)

    bev1, _, _ = cam.getBevFromImg_1(img)
    bev2, _, _ = cam.getBevFromImg_2(img)
    bev3 = cam.getBevFromImg_3(img)

    # for i in range(4):
    #     cv2.circle(img, tuple(pts_img[i]), 5, (0, 0, 255))
    #     cv2.circle(bev1, tuple(pts_bev[i]), 5, (0, 0, 255))
    #     cv2.circle(bev2, tuple(pts_bev[i]), 5, (0, 0, 255))
    #     cv2.circle(bev3, tuple(pts_bev[i]), 5, (0, 0, 255))
    cv2.imshow('img', img)
    cv2.imshow('bev1', bev1)
    cv2.imshow('bev2', bev2)
    cv2.imshow('bev3', bev3)

    cv2.waitKey(0)