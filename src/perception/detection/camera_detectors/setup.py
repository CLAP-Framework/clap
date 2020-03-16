#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['yolov3', 'yolov3.utils', 'zzz_perception_detection_camera_detectors'],
    package_dir={'': 'src'},
    package_data={'yolov3': ['src/yolov3/cfg/*', 'src/yolov3/data/*', 'src/yolov3/weights/*']},
    install_requires={'torch', 'pathlib', 'google-cloud-storage'}
)

setup(**d)