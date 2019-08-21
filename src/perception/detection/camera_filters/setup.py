#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['zzz_perception_detection_camera_filters', 'pseudo_lidar'],
    package_dir={'': 'src'},
    install_requires=['numpy']
)

setup(**d)
