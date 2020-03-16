#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['zzz_perception_detection_lidar_filters'],
    package_dir={'': 'src'},
    install_requires={'pcl-py'}
)

setup(**d)