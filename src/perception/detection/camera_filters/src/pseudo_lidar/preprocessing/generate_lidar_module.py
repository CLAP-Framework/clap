import numpy as np


class PclGenerator(object):
    def __init__(self, params):
        self.args = params

    def run(self, calib, disp, baseline=0.54):
        # baseline=0.54 is for KITTI (https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT)
        disp[disp < 0] = 0
        mask = disp > 0
        depth = calib.f_u * baseline / (disp + 1. - mask)
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        points = np.stack([c, r, depth])
        points = points.reshape((3, -1))
        points = points.T
        points = points[mask.reshape(-1)]
        cloud = calib.project_image_to_velo(points)
        valid = (cloud[:, 0] >= 0) & (cloud[:, 2] < self.args.max_high)
        # valid = (cloud[:, 0] > 0)
        # valid = np.ones(cloud.shape[0]) == 1
        
        return cloud[valid], valid



def main():
    generate_lidar_node = PclGenerator()
    generate_lidar_node.run_from_file()

if __name__ == '__main__':
    main()