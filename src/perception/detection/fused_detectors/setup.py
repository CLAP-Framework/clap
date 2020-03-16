#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['frustum_pointnets',
        'frustum_pointnets.train',
        'frustum_pointnets.module_wrapper'],
        # 'zzz_perception_detection_fused_detectors'],
    package_dir={'': 'src'},
    # package_data={'yolov3': ['src/yolov3/cfg/*', 'src/yolov3/data/*', 'src/yolov3/weights/*']},
    install_requires={'tensorflow'}
)

setup(**d)