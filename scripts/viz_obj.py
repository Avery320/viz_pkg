#!/usr/bin/env python3

import os
import sys

# Ensure local src/ is on sys.path so flat modules can be imported in devel space
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', 'src'))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)


import rospy

# Entry-point script that uses the reusable module
from obj_loader import ObjPublisher


if __name__ == '__main__':
    rospy.init_node('mesh_object_publisher')

    # One-shot behavior: load objects and exit after a short wait
    node = ObjPublisher()
    rospy.loginfo("One-shot: waiting for collision objects to be added...")
    rospy.sleep(3.0)  # Allow time for MoveIt collision objects and at least one marker publish
    rospy.loginfo("One-shot: done, exiting")
