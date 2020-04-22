"""Provides helper methods for loading and parsing KITTI data."""

import os
import datetime
from collections import namedtuple

import numpy as np
from PIL import Image
import xml.etree.ElementTree as ET

# Per dataformat.txt
_OxtsPacket = namedtuple('OxtsPacket',
                        'lat, lon, alt, ' +
                        'roll, pitch, yaw, ' +
                        'vn, ve, vf, vl, vu, ' +
                        'ax, ay, az, af, al, au, ' +
                        'wx, wy, wz, wf, wl, wu, ' +
                        'pos_accuracy, vel_accuracy, ' +
                        'navstat, numsats, ' +
                        'posmode, velmode, orimode')

# Bundle into an easy-to-access structure
_OxtsData = namedtuple('OxtsData', 'packet, T_w_imu')

def rotx(t):
    """Rotation about the x-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])


def roty(t):
    """Rotation about the y-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])


def rotz(t):
    """Rotation about the z-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])

def rt(R, T):
    """Transforation matrix from rotation matrix and translation vector."""
    rt = np.eye(4)
    rt[:3,:3] = R.reshape(3, 3)
    rt[:3,3] = T.reshape(3)
    return rt

def _pose_from_oxts_packet(packet, scale):
    """Helper method to compute a SE(3) pose matrix from an OXTS packet.
    """
    er = 6378137.  # earth radius (approx.) in meters

    # Use a Mercator projection to get the translation vector
    tx = scale * packet.lon * np.pi * er / 180.
    ty = scale * er * np.log(np.tan((90. + packet.lat) * np.pi / 360.))
    tz = packet.alt
    t = np.array([tx, ty, tz])

    # Use the Euler angles to get the rotation matrix
    Rx = rotx(packet.roll)
    Ry = roty(packet.pitch)
    Rz = rotz(packet.yaw)
    R = Rz.dot(Ry.dot(Rx))

    # Combine the translation and rotation into a homogeneous transform
    return R, t


def load_oxts_packets(basepath, oxts_files):
    """Generator to read OXTS ground truth data.

       Poses are given in an East-North-Up coordinate system 
       whose origin is the first GPS position.
    """
    # Scale for Mercator projection (from first lat value)
    scale = None
    # Origin of the global coordinate system (first GPS position)
    origin = None

    oxts = []

    for filename in oxts_files:
        if isinstance(basepath, str):
            fin = open(os.path.join(basepath, filename))
        else: # assume ZipFile object
            fin = basepath.open(filename)
        with fin:
            for line in fin.readlines():
                line = line.split()
                # Last five entries are flags and counts
                line[:-5] = [float(x) for x in line[:-5]]
                line[-5:] = [int(float(x)) for x in line[-5:]]

                packet = _OxtsPacket(*line)

                if scale is None:
                    scale = np.cos(packet.lat * np.pi / 180.)

                R, t = _pose_from_oxts_packet(packet, scale)

                if origin is None:
                    origin = t

                T_w_imu = rt(R, t - origin)

                oxts.append(_OxtsData(packet, T_w_imu))

    return oxts

# ========== File Loaders ========== #

def load_timestamps(basepath, file, formatted=False):
    """
    Read in timestamp file and parse to a list
    """
    timestamps = []
    if isinstance(basepath, str):
        fin = open(os.path.join(basepath, file))
    else: # assume ZipFile object
        fin = basepath.open(file)

    with fin:
        if formatted:
            for line in fin.readlines():
                timestamps.append(np.datetime64(line))
        else:
            timestamps = (np.loadtxt(fin) * 1e9).astype("M8[ns]")

    return timestamps

def load_calib_file(basepath, file):
    """
    Read in a calibration file and parse into a dictionary.
    Accept path or file object as input
    """
    data = {}
    if isinstance(basepath, str):
        fin = open(os.path.join(basepath, file))
    else: # assume ZipFile object
        fin = basepath.open(file)

    with fin:
        for line in fin.readlines():
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data

def load_image(basepath, file, gray=False):
    """Load an image from file. Accept path or file object as basepath"""
    if isinstance(basepath, str):
        return Image.open(os.path.join(basepath, file)).convert('L' if gray else 'RGB')
    else: # assume ZipFile object
        return Image.open(basepath.open(file)).convert('L' if gray else 'RGB')

def yield_images(basepath, filelist, gray=False):
    """Generator to read image files."""
    for file in filelist:
        yield load_image(basepath, file, gray)

def load_velo_scan(basepath, file, binary=True):
    """Load and parse a kitti file. Accept path or file object as basepath"""
    if binary:
        if isinstance(basepath, str):
            scan = np.fromfile(os.path.join(basepath, file), dtype=np.float32)
        else:
            with basepath.open(file) as fin:
                buffer = fin.read()
            scan = np.frombuffer(buffer, dtype=np.float32)
    else:
        if isinstance(basepath, str):
            scan = np.loadtxt(os.path.join(basepath, file), dtype=np.float32)
        else:
            scan = np.loadtxt(basepath.open(file), dtype=np.float32)
    return scan.reshape((-1, 4))

def yield_velo_scans(basepath, filelist, binary=True):
    """Generator to parse velodyne files into arrays."""
    for file in filelist:
        yield load_velo_scan(basepath, file, binary=binary)

class _TrackletPose(object):
    def __init__(self, xmlnode):
        for prop in xmlnode:
            setattr(self, prop.tag, float(prop.text))

class _TrackletObject(object):
    def __init__(self, xmlnode):
        for prop in xmlnode:
            if prop.tag == 'poses':
                self.poses = [_TrackletPose(item) for item in prop if item.tag == 'item']
            elif prop.tag == "objectType":
                self.objectType = prop.text
            else:
                setattr(self, prop.tag, float(prop.text))

def load_tracklets(basepath, file):
    if isinstance(basepath, str):
        fin = open(os.path.join(basepath, file))
    else: # assume ZipFile object
        fin = basepath.open(file)

    with fin:
        root = ET.fromstring(fin.read())
        root_tracklet = next(iter(root))
        tracklets = [_TrackletObject(item) for item in root_tracklet if item.tag == 'item']
        return tracklets
