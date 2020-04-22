"""This module loads and parses odometry benchmark data."""

from zipfile import ZipFile
import os
import os.path as osp

import numpy as np
import zzz_driver_datasets_kitti.utils as utils

class OdometryDataset:
    """
    Load and parse odometry benchmark data into a usable format.
    Please ensure that calibration and poses are downloaded
    
    # Zip Files
    - data_odometry_calib.zip [required]
    - data_odometry_color.zip
    - data_odometry_gray.zip
    - data_odometry_velodyne.zip
    - data_odometry_poses.zip [required]

    # Unzipped Structure
    - <base_path directory>
        - dataset
            - poses
                - 00.txt
                - ..
            - sequences
                - 00
                    - image_0
                    - image_1
                    - image_2
                    - image_3
                    - velodyne_points
                    - calib.txt
                    - times.txt
    """

    def __init__(self, base_path, sequence, frames=None, inzip=False):
        """Set the path."""
        self.seqname = "%02d" % int(sequence)
        self.base_path = base_path
        self.frames = frames
        self.inzip = inzip

        self.pose_data = None
        self.gray_data = None
        self.color_data = None
        self.velo_data = None

        try:
            if inzip:
                assert osp.exists(osp.join(base_path, "data_odometry_calib.zip"))
                assert osp.exists(osp.join(base_path, "data_odometry_poses.zip"))
            else:
                assert osp.exists(osp.join(base_path, "dataset", "sequences", self.seqname))
                assert osp.exists(osp.join(base_path, "dataset", "poses"))
        except AssertionError:
            raise FileNotFoundError("Cannot find proper files!")

        if self.inzip:
            self.pose_data = ZipFile(osp.join(base_path, "data_odometry_poses.zip"))
            if osp.exists(osp.join(base_path, "data_odometry_gray.zip")):
                self.gray_data = ZipFile(osp.join(base_path, "data_odometry_gray.zip"))
            if osp.exists(osp.join(base_path, "data_odometry_color.zip")):
                self.color_data = ZipFile(osp.join(base_path, "data_odometry_color.zip"))
            if osp.exists(osp.join(base_path, "data_odometry_velodyne.zip")):
                self.velo_data = ZipFile(osp.join(base_path, "data_odometry_velodyne.zip"))
        else:
            self.sequence_path = os.path.join(base_path, "dataset", 'sequences', self.seqname)
            self.pose_path = os.path.join(base_path, "dataset", 'poses')

        # Find all the data files
        self._get_file_lists()

        # Pre-load data that isn't returned as a generator
        self._load_calib()
        self._load_timestamps()
        self._load_poses()

    def __len__(self):
        """Return the number of frames loaded."""
        return len(self.timestamps)

    def close(self):
        if self.inzip:
            self.pose_data.close()
            if self.gray_data is not None:
                self.gray_data.close()
            if self.color_data is not None:
                self.color_data.close()
            if self.velo_data is not None:
                self.velo_data.close()

    # ========== Generators ==========

    @property
    def cam0(self):
        """Generator to read image files for cam0 (monochrome left)."""
        return utils.yield_images(self.gray_data if self.inzip else self.sequence_path, self.cam0_files, gray=True)
    @property
    def cam1(self):
        """Generator to read image files for cam1 (monochrome right)."""
        return utils.yield_images(self.gray_data if self.inzip else self.sequence_path, self.cam1_files, gray=True)
    @property
    def cam2(self):
        """Generator to read image files for cam2 (RGB left)."""
        return utils.yield_images(self.color_data if self.inzip else self.sequence_path, self.cam2_files, gray=False)
    @property
    def cam3(self):
        """Generator to read image files for cam3 (RGB right)."""
        return utils.yield_images(self.color_data if self.inzip else self.sequence_path, self.cam3_files, gray=False)
    @property
    def velo(self):
        """Generator to read velodyne [x,y,z,intensity/reflectance] scan data from binary files."""
        return utils.yield_velo_scans(self.velo_data if self.inzip else self.sequence_path, self.velo_files)

    # ========== Implementations ==========

    def _get_file_lists(self):
        """Find and list data files for each sensor."""
        if self.inzip:
            sname_offset = len("dataset/sequences/")
            dname_offset = len("dataset/sequences/00/")
            self.cam0_files = []
            self.cam1_files = []
            self.cam2_files = []
            self.cam3_files = []
            self.velo_files = []

            if self.gray_data is not None:
                for fname in self.gray_data.namelist():
                    if fname.endswith('/') or fname[sname_offset:sname_offset + 2] != self.seqname:
                        continue
                    if fname[dname_offset:dname_offset + 7] == 'image_0':
                        self.cam0_files.append(fname)
                    elif fname[dname_offset:dname_offset + 7] == 'image_1':
                        self.cam1_files.append(fname)
            if self.color_data is not None:
                for fname in self.color_data.namelist():
                    if fname.endswith('/') or fname[sname_offset:sname_offset + 2] != self.seqname:
                        continue
                    if fname[dname_offset:dname_offset + 7] == 'image_2':
                        self.cam2_files.append(fname)
                    elif fname[dname_offset:dname_offset + 7] == 'image_3':
                        self.cam3_files.append(fname)
            if self.velo_data is not None:
                for fname in self.color_data.namelist():
                    if fname.endswith('/') or fname[sname_offset:sname_offset + 2] != self.seqname:
                        continue
                # TODO: Implement velo_data part

            self.cam0_files = sorted(self.cam0_files)
            self.cam1_files = sorted(self.cam1_files)
            self.cam2_files = sorted(self.cam2_files)
            self.cam3_files = sorted(self.cam3_files)
            self.velo_files = sorted(self.velo_files)

        else:
            self.cam0_files = sorted(osp.join('image_0', fname) for fname
                in os.listdir(osp.join(self.sequence_path, 'image_0'))) # if fname.endswith(".png"))
            self.cam1_files = sorted(osp.join('image_1', fname) for fname
                in os.listdir(osp.join(self.sequence_path, 'image_1'))) # if fname.endswith(".png"))
            self.cam2_files = sorted(osp.join('image_2', fname) for fname
                in os.listdir(osp.join(self.sequence_path, 'image_2'))) # if fname.endswith(".png"))
            self.cam3_files = sorted(osp.join('image_3', fname) for fname
                in os.listdir(osp.join(self.sequence_path, 'image_3'))) # if fname.endswith(".png"))
            self.velo_files = sorted(osp.join('velodyne_points', fname) for fname
                in os.listdir(osp.join(self.sequence_path, 'velodyne_points'))) # if fname.endswith(".bin"))

        # Subselect the chosen range of frames, if any
        if self.frames is not None:
            self.cam0_files = [self.cam0_files[i] for i in self.frames]
            self.cam1_files = [self.cam1_files[i] for i in self.frames]
            self.cam2_files = [self.cam2_files[i] for i in self.frames]
            self.cam3_files = [self.cam3_files[i] for i in self.frames]
            self.velo_files = [self.velo_files[i] for i in self.frames]

    def _load_calib(self):
        """Load and compute intrinsic and extrinsic calibration parameters."""
        # We'll build the calibration parameters as a dictionary, then
        # convert it to a namedtuple to prevent it from being modified later
        data = {}
        calib_path = ZipFile(osp.join(self.base_path, "data_odometry_calib.zip")) \
            if self.inzip else self.base_path

        # Load the calibration file
        filedata = utils.load_calib_file(calib_path, "dataset/sequences/{}/calib.txt".format(self.seqname))

        # Create 3x4 projection matrices
        P_rect_00 = filedata['P0'].reshape(3, 4)
        P_rect_01 = filedata['P1'].reshape(3, 4)
        P_rect_02 = filedata['P2'].reshape(3, 4)
        P_rect_03 = filedata['P3'].reshape(3, 4)
        data['P0'] = P_rect_00
        data['P1'] = P_rect_01
        data['P2'] = P_rect_02
        data['P3'] = P_rect_03

        # Compute the rectified extrinsics from cam0 to camN
        T1 = np.eye(4)
        T1[0, 3] = P_rect_01[0, 3] / P_rect_01[0, 0]
        T2 = np.eye(4)
        T2[0, 3] = P_rect_02[0, 3] / P_rect_02[0, 0]
        T3 = np.eye(4)
        T3[0, 3] = P_rect_03[0, 3] / P_rect_03[0, 0]

        # Compute the velodyne to rectified camera coordinate transforms
        data['T_cam0_velo'] = np.vstack([filedata['Tr'].reshape(3, 4), [0, 0, 0, 1]])
        data['T_cam1_velo'] = T1.dot(data['T_cam0_velo'])
        data['T_cam2_velo'] = T2.dot(data['T_cam0_velo'])
        data['T_cam3_velo'] = T3.dot(data['T_cam0_velo'])

        # Compute the stereo baselines in meters by projecting the origin of
        # each camera frame into the velodyne frame and computing the distances
        # between them
        p_cam = np.array([0, 0, 0, 1])
        p_velo0 = np.linalg.inv(data['T_cam0_velo']).dot(p_cam)
        p_velo1 = np.linalg.inv(data['T_cam1_velo']).dot(p_cam)
        p_velo2 = np.linalg.inv(data['T_cam2_velo']).dot(p_cam)
        p_velo3 = np.linalg.inv(data['T_cam3_velo']).dot(p_cam)

        data['b_gray'] = np.linalg.norm(p_velo1 - p_velo0)  # gray baseline
        data['b_rgb'] = np.linalg.norm(p_velo3 - p_velo2)   # rgb baseline

        self.calib = data
        # Close calib file if needed
        if self.inzip:
            calib_path.close()

    def _load_timestamps(self):
        """Load timestamps from file."""
        data_path = ZipFile(osp.join(self.base_path, "data_odometry_calib.zip")) if self.inzip else self.base_path
        self.timestamps = utils.load_timestamps(data_path, "dataset/sequences/{}/times.txt".format(self.seqname))

        # Subselect the chosen range of frames, if any
        if self.frames is not None:
            self.timestamps = [self.timestamps[i] for i in self.frames]

        # Soft copy timestamps
        self.cam0_timestamps = self.cam1_timestamps\
            = self.cam2_timestamps = self.cam3_timestamps\
            = self.velo_timestamps = self.timestamps

        # Close calib file if needed
        if self.inzip:
            data_path.close()

    def _load_poses(self):
        """Load ground truth poses (T_w_cam0) from file."""
        self.poses = []
        try:
            if self.inzip:
                data_path = ZipFile(osp.join(self.base_path, "data_odometry_poses.zip"))
                data_file = data_path.open("dataset/poses/{}.txt".format(self.seqname))
            else:
                data_file = open(os.path.join(self.pose_path, self.seqname + '.txt'))
        except: # Sequence with no poses ground truth
            if self.inzip:
                data_path.close()
            return

        with data_file:
            lines = data_file.readlines()
            if self.frames is not None:
                lines = [lines[i] for i in self.frames]

            for line in lines:
                T_w_cam0 = np.fromstring(line, dtype=float, sep=' ')
                T_w_cam0 = T_w_cam0.reshape(3, 4)
                T_w_cam0 = np.vstack((T_w_cam0, [0, 0, 0, 1]))
                self.poses.append(T_w_cam0)
        
        if self.inzip:
            data_path.close()
