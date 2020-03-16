#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['zzz_perception_tracking_object_trackers'],
    package_dir={'': 'src'},
    install_requires=['scipy', 'filterpy']
)

setup(**d)