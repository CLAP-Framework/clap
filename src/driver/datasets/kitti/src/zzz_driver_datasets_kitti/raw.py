"""This module which loads and parses raw KITTI data."""

from zipfile import ZipFile
import os
import os.path as osp

import numpy as np
import zzz_driver_datasets_kitti.utils as utils

class RawDataset:
    """
    Load and parse raw data into a usable format.

    # Zip Files
    - 2011_09_26_calib.zip [required]
    - 2011_09_26_drive_0001_extract.zip
    - ...
    - 2011_09_26_drive_0001_sync.zip
    - ...
    - 2011_09_26_drive_0001_tracklets.zip
    - ...

    # Unzipped Structure
    - <base_path directory>
        - 2011_09_26
            - calib_cam_to_cam.txt
            - calib_imu_to_velo.txt
            - calib_velo_to_cam.txt
            - 2011_09_26_drive_0001_extract
                - image_00
                - image_01
                - image_02
                - image_03
                - oxts
                - velodyne_points
            - ...
            - 2011_09_26_drive_0001_sync
                - image_00
                - image_01
                - image_02
                - image_03
                - oxts
                - velodyne_points
                - tracklet_labels.xml
            - ...
            - 
    """
    def __init__(self, base_path, drive, datatype='sync', frames=None, inzip=False):
        """
        Set the path and pre-load calibration data and timestamps.

        # Parameters
        - datatype: 'sync' (synced) / 'extract' (unsynced)
        - drive: drive sequence number
        """
        drive = "%04d" % int(drive)
        self.data_type = datatype
        self.seqname = '2011_09_26_drive_' + drive + '_' + datatype
        self.base_path = base_path
        self.frames = frames
        self.inzip = inzip

        try:
            if inzip:
                assert osp.exists(osp.join(base_path, "2011_09_26_calib.zip"))
                assert osp.exists(osp.join(base_path, self.seqname + ".zip"))
            else:
                assert osp.exists(osp.join(base_path, "2011_09_26", "calib_cam_to_cam.txt"))
                assert osp.exists(osp.join(base_path, "2011_09_26", self.seqname))
        except AssertionError:
            raise FileNotFoundError("Cannot find proper files!")

        # Preopen data files
        if self.inzip:
            self.data_file = ZipFile(osp.join(base_path, self.seqname + ".zip"))
        else:
            self.data_path = osp.join(base_path, "2011_09_26", self.seqname)

        # Find all the data files
        self._get_file_lists()

        # Pre-load data that isn't returned as a generator
        self._load_calib()
        self._load_timestamps()
        self._load_oxts()
        if datatype == "sync": self._load_tracklets()

    def __len__(self):
        """Return the number of frames loaded."""
        if self.data_type != "sync":
            raise ValueError("The length of unsynced data is ambiguous!")
        return len(self.oxts_timestamps)

    def close(self):
        if self.inzip:
            self.data_file.close()

    # ========== Generators ==========

    @property
    def cam0(self):
        """Generator to read image files for cam0 (monochrome left)."""
        return utils.yield_images(self.data_file if self.inzip else self.data_path, self.cam0_files, gray=True)
    @property
    def cam1(self):
        """Generator to read image files for cam1 (monochrome right)."""
        return utils.yield_images(self.data_file if self.inzip else self.data_path, self.cam1_files, gray=True)
    @property
    def cam2(self):
        """Generator to read image files for cam2 (RGB left)."""
        return utils.yield_images(self.data_file if self.inzip else self.data_path, self.cam2_files, gray=False)
    @property
    def cam3(self):
        """Generator to read image files for cam3 (RGB right)."""
        return utils.yield_images(self.data_file if self.inzip else self.data_path, self.cam3_files, gray=False)
    @property
    def velo(self):
        """Generator to read velodyne [x,y,z,intensity/reflectance] scan data from files."""
        return utils.yield_velo_scans(self.data_file if self.inzip else self.data_path, self.velo_files, binary=(self.data_type == "sync"))

    # ========== Implementations ==========

    def _get_file_lists(self):
        """Find and list data files for each sensor."""
        # Read file names from zip file
        root_folder = "2011_09_26/"
        if self.inzip:
            start_offset = len(root_folder) + len(self.seqname) + 1

            self.oxts_files = []
            self.cam0_files = []
            self.cam1_files = []
            self.cam2_files = []
            self.cam3_files = []
            self.velo_files = []

            for fname in self.data_file.namelist():
                if fname.endswith('/'):
                    continue
                if fname[start_offset:start_offset + 10] == 'oxts/data/':
                    self.oxts_files.append(fname)
                elif fname[start_offset:start_offset + 14] == 'image_00/data/':
                    self.cam0_files.append(fname)
                elif fname[start_offset:start_offset + 14] == 'image_01/data/':
                    self.cam1_files.append(fname)
                elif fname[start_offset:start_offset + 14] == 'image_02/data/':
                    self.cam2_files.append(fname)
                elif fname[start_offset:start_offset + 14] == 'image_03/data/':
                    self.cam3_files.append(fname)
                elif fname[start_offset:start_offset + 21] == 'velodyne_points/data/':
                    self.velo_files.append(fname)

            self.oxts_files = sorted(self.oxts_files)
            self.cam0_files = sorted(self.cam0_files)
            self.cam1_files = sorted(self.cam1_files)
            self.cam2_files = sorted(self.cam2_files)
            self.cam3_files = sorted(self.cam3_files)
            self.velo_files = sorted(self.velo_files)
                        
            self.oxts_time_file = root_folder + self.seqname + "/oxts/timestamps.txt"
            self.cam0_time_file = root_folder + self.seqname + "/image_00/timestamps.txt"
            self.cam1_time_file = root_folder + self.seqname + "/image_01/timestamps.txt"
            self.cam2_time_file = root_folder + self.seqname + "/image_02/timestamps.txt"
            self.cam3_time_file = root_folder + self.seqname + "/image_03/timestamps.txt"
            self.velo_time_file = root_folder + self.seqname + "/velodyne_points/timestamps_end.txt"

        # Read file names from directory
        else:
            self.oxts_files = sorted(osp.join('oxts', 'data', fname) for fname
                in os.listdir(osp.join(self.data_path, 'oxts', 'data'))) # if fname.endswith(".txt"))
            self.cam0_files = sorted(osp.join('image_00', 'data', fname) for fname
                in os.listdir(osp.join(self.data_path, 'image_00', 'data'))) # if fname.endswith(".png"))
            self.cam1_files = sorted(osp.join('image_01', 'data', fname) for fname
                in os.listdir(osp.join(self.data_path, 'image_01', 'data'))) # if fname.endswith(".png"))
            self.cam2_files = sorted(osp.join('image_02', 'data', fname) for fname
                in os.listdir(osp.join(self.data_path, 'image_02', 'data'))) # if fname.endswith(".png"))
            self.cam3_files = sorted(osp.join('image_03', 'data', fname) for fname
                in os.listdir(osp.join(self.data_path, 'image_03', 'data'))) # if fname.endswith(".png"))
            self.velo_files = sorted(osp.join('velodyne_points', 'data', fname) for fname
                in os.listdir(osp.join(self.data_path, 'velodyne_points', 'data'))) # if fname.endswith(".bin"))

            self.oxts_time_file = osp.join('oxts', 'timestamps.txt')
            self.cam0_time_file = osp.join('image_00', 'timestamps.txt')
            self.cam1_time_file = osp.join('image_01', 'timestamps.txt')
            self.cam2_time_file = osp.join('image_02', 'timestamps.txt')
            self.cam3_time_file = osp.join('image_03', 'timestamps.txt')
            self.velo_time_file = osp.join('velodyne_points', 'timestamps.txt')

        # Subselect the chosen range of frames, if any
        if self.frames is not None:
            self.oxts_files = [self.oxts_files[i] for i in self.frames]
            self.cam0_files = [self.cam0_files[i] for i in self.frames]
            self.cam1_files = [self.cam1_files[i] for i in self.frames]
            self.cam2_files = [self.cam2_files[i] for i in self.frames]
            self.cam3_files = [self.cam3_files[i] for i in self.frames]
            self.velo_files = [self.velo_files[i] for i in self.frames]

    def _load_calib(self):
        """Load and compute intrinsic and extrinsic calibration parameters."""
        data = {}
        root_folder = "2011_09_26/"
        calib_path = ZipFile(osp.join(self.base_path, "2011_09_26_calib.zip")) \
            if self.inzip else self.base_path


        # ----- Load the rigid transformation from IMU to velodyne -----
        imu_to_velo_data = utils.load_calib_file(calib_path, root_folder + "calib_imu_to_velo.txt")
        data['T_velo_imu'] = utils.rt(imu_to_velo_data['R'], imu_to_velo_data['T'])


        # ----- Load the camera intrinsics and extrinsics -----
        # Load the rigid transformation from velodyne coordinates to unrectified cam0 coordinates
        cam_to_cam_data = utils.load_calib_file(calib_path, root_folder + "calib_velo_to_cam.txt")
        T_cam0unrect_velo = utils.rt(cam_to_cam_data['R'], cam_to_cam_data['T'])
        data['T_cam0_velo_unrect'] = T_cam0unrect_velo


        # ----- Load and parse the cam-to-cam calibration data -----
        cam_to_cam_data = utils.load_calib_file(calib_path, root_folder + "calib_cam_to_cam.txt")
        data.update(cam_to_cam_data)

        # Create 3x4 projection matrices
        P_rect_00 = cam_to_cam_data['P_rect_00'].reshape(3, 4)
        P_rect_01 = cam_to_cam_data['P_rect_01'].reshape(3, 4)
        P_rect_02 = cam_to_cam_data['P_rect_02'].reshape(3, 4)
        P_rect_03 = cam_to_cam_data['P_rect_03'].reshape(3, 4)
        data['P_rect_00'] = P_rect_00
        data['P_rect_01'] = P_rect_01
        data['P_rect_02'] = P_rect_02
        data['P_rect_03'] = P_rect_03

        # Create 4x4 matrices from the rectifying rotation matrices
        data['R_rect_00'] = cam_to_cam_data['R_rect_00'].reshape(3, 3)
        data['R_rect_01'] = cam_to_cam_data['R_rect_01'].reshape(3, 3)
        data['R_rect_02'] = cam_to_cam_data['R_rect_02'].reshape(3, 3)
        data['R_rect_03'] = cam_to_cam_data['R_rect_03'].reshape(3, 3)


        # ----- Compute auxillary parameters ----
        # Compute the rectified extrinsics from cam0 to camN
        T0 = np.eye(4)
        T0[0, 3] = P_rect_00[0, 3] / P_rect_00[0, 0]
        T1 = np.eye(4)
        T1[0, 3] = P_rect_01[0, 3] / P_rect_01[0, 0]
        T2 = np.eye(4)
        T2[0, 3] = P_rect_02[0, 3] / P_rect_02[0, 0]
        T3 = np.eye(4)
        T3[0, 3] = P_rect_03[0, 3] / P_rect_03[0, 0]

        # Compute the velodyne to rectified camera coordinate transforms
        R_rect_00 = np.eye(4)
        R_rect_00[0:3, 0:3] = data['R_rect_00']
        R_rect_01 = np.eye(4)
        R_rect_01[0:3, 0:3] = data['R_rect_01']
        R_rect_02 = np.eye(4)
        R_rect_02[0:3, 0:3] = data['R_rect_02']
        R_rect_03 = np.eye(4)
        R_rect_03[0:3, 0:3] = data['R_rect_03']
        data['T_cam0_velo'] = T0.dot(R_rect_00.dot(T_cam0unrect_velo))
        data['T_cam1_velo'] = T1.dot(R_rect_01.dot(T_cam0unrect_velo))
        data['T_cam2_velo'] = T2.dot(R_rect_02.dot(T_cam0unrect_velo))
        data['T_cam3_velo'] = T3.dot(R_rect_03.dot(T_cam0unrect_velo))

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

        # Pre-compute the IMU to rectified camera coordinate transforms
        data['T_cam0_imu'] = data['T_cam0_velo'].dot(data['T_velo_imu'])
        data['T_cam1_imu'] = data['T_cam1_velo'].dot(data['T_velo_imu'])
        data['T_cam2_imu'] = data['T_cam2_velo'].dot(data['T_velo_imu'])
        data['T_cam3_imu'] = data['T_cam3_velo'].dot(data['T_velo_imu'])

        self.calib = data
        # Close calib file if needed
        if self.inzip:
            calib_path.close()

    def _load_timestamps(self):
        """Load timestamps from file."""
        data_path = self.data_file if self.inzip else self.data_path

        # Read and parse the timestamps
        self.oxts_timestamps = utils.load_timestamps(data_path, self.oxts_time_file, formatted=True)
        self.cam0_timestamps = utils.load_timestamps(data_path, self.cam0_time_file, formatted=True)
        self.cam1_timestamps = utils.load_timestamps(data_path, self.cam1_time_file, formatted=True)
        self.cam2_timestamps = utils.load_timestamps(data_path, self.cam2_time_file, formatted=True)
        self.cam3_timestamps = utils.load_timestamps(data_path, self.cam3_time_file, formatted=True)
        self.velo_timestamps = utils.load_timestamps(data_path, self.velo_time_file, formatted=True)

        # Subselect the chosen range of frames, if any
        if self.frames is not None:
            self.oxts_timestamps = [self.oxts_timestamps[i] for i in self.frames]
            self.cam0_timestamps = [self.cam0_timestamps[i] for i in self.frames]
            self.cam1_timestamps = [self.cam1_timestamps[i] for i in self.frames]
            self.cam2_timestamps = [self.cam2_timestamps[i] for i in self.frames]
            self.cam3_timestamps = [self.cam3_timestamps[i] for i in self.frames]
            self.velo_timestamps = [self.velo_timestamps[i] for i in self.frames]

    def _load_oxts(self):
        """Load OXTS data from file."""
        data_path = self.data_file if self.inzip else self.data_path
        self.oxts = utils.load_oxts_packets(data_path, self.oxts_files)

    def _load_tracklets(self):
        """Load tracklets from file. Note: tracklets stamps are assumed to be the same as point clouds'"""
        data_path = self.data_file if self.inzip else self.data_path
        tracklets = utils.load_tracklets(data_path, "tracklet_labels.xml")
        tracklet_frames = [[] for i in range(len(self.velo_timestamps))] # Create array
        for track in tracklets:
            for idx, pose in enumerate(track.poses):
                pose.objectType = track.objectType
                pose.h = track.h
                pose.w = track.w
                pose.l = track.l
                pose.id = idx
                tracklet_frames[idx + int(track.first_frame)].append(pose)
        self.tracklets = tracklet_frames
