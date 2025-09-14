#!/usr/bin/env python3

import sys
import rospy

# Optional MoveIt integration
try:
    import moveit_commander
    from moveit_commander import PlanningSceneInterface
    MOVEIT_AVAILABLE = True
except Exception as e:
    MOVEIT_AVAILABLE = False
    _MOVEIT_IMPORT_ERROR = e


def main(argv):
    rospy.init_node('list_models', anonymous=True)
    if not MOVEIT_AVAILABLE:
        rospy.logerr(f"moveit_commander not available: {_MOVEIT_IMPORT_ERROR}")
        return 1
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)
        names = scene.get_known_object_names()
        if not names:
            print('(no known world objects)')
        else:
            for n in names:
                print(n)
        return 0
    except Exception as e:
        rospy.logerr(f"Failed to list models: {e}")
        return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))

