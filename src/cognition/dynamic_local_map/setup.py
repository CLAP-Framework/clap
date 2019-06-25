#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['zzz_cognition_dynamic_local_map'],
    package_dir={'': 'src'}
)

setup(**d)