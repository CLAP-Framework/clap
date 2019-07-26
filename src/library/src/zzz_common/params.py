'''
This module provides consistent params loading from rosparam and command line input
'''

import argparse
import rospy
from easydict import EasyDict as edict

# TODO: create a fake ArgumentParser to show additional information and support complex arguments
def parse_private_args(**kvargs):
    parser = argparse.ArgumentParser()
    node_name = rospy.get_name()

    ros_args = {}
    for name, default in kvargs.items():
        parser.add_argument(name, default=default, nargs='?')
        ros_args[name] = rospy.get_param(node_name + '/' + name, default)
    cmd_args = parser.parse_args()

    # XXX: This will override existing args
    # ros_args.update(var(cmd_args))
    return edict(ros_args)
