#!/usr/bin/env python3

import os
import sys

# Ensure local src/ is on sys.path so flat modules can be imported in devel space
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', 'src'))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)


import rospy
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

# Entry-point script that uses the reusable module
from obj_loader import ObjPublisher


if __name__ == '__main__':
    rospy.init_node('mesh_object_publisher')
    # Republish MoveIt's monitored PlanningScene to a stable, latched topic
    _ps_pub = rospy.Publisher('/planning_scene/scene', PlanningScene, queue_size=1, latch=True)
    rospy.Subscriber('/move_group/monitored_planning_scene', PlanningScene, lambda m: _ps_pub.publish(m), queue_size=1)
    rospy.loginfo("Republishing PlanningScene: /move_group/monitored_planning_scene -> /planning_scene/scene (latched)")

    # Publish an initial full snapshot so late subscribers immediately get the world objects
    try:
        rospy.wait_for_service('/get_planning_scene', timeout=5.0)
        get_ps = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        comp = PlanningSceneComponents()
        comp.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                           PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        resp = get_ps(comp)
        _ps_pub.publish(resp.scene)
        rospy.loginfo("Published initial PlanningScene snapshot to /planning_scene/scene")
    except Exception as e:
        rospy.logwarn(f"Failed to fetch initial PlanningScene: {e}")

    # Keep-alive behavior: load objects and keep node running
    node = ObjPublisher()
    rospy.loginfo("Keep-alive: viz_obj is running. Republish PlanningScene on /planning_scene/scene; publishing markers on /env_marker")
    rospy.spin()
