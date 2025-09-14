#!/usr/bin/env python3

import sys
import rospy
from visualization_msgs.msg import Marker

# Try MoveIt (optional)
try:
    import moveit_commander
    from moveit_commander import PlanningSceneInterface
    MOVEIT_AVAILABLE = True
except Exception as e:
    MOVEIT_AVAILABLE = False
    _MOVEIT_IMPORT_ERROR = e


def clear_moveit_objects(object_id: str, remove_all: bool):
    if not MOVEIT_AVAILABLE:
        rospy.logwarn(f"moveit_commander not available; skip removing collision objects (error: {_MOVEIT_IMPORT_ERROR})")
        return
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(1.0)
        if remove_all:
            try:
                names = scene.get_known_object_names()
            except Exception:
                names = []
            if not names:
                rospy.loginfo('No known world objects to remove')
            for n in names:
                scene.remove_world_object(n)
                rospy.loginfo(f'Removed world object: {n}')
        else:
            scene.remove_world_object(object_id)
            rospy.loginfo(f'Removed world object: {object_id}')
    except Exception as e:
        rospy.logerr(f'Failed to remove MoveIt world objects: {e}')


def clear_rviz_markers(topic: str, frame_id: str):
    pub = rospy.Publisher(topic, Marker, queue_size=1, latch=False)
    # wait a moment for connection
    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and (rospy.Time.now() - t0).to_sec() < 1.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.action = Marker.DELETEALL
    for _ in range(3):
        pub.publish(m)
        rospy.sleep(0.1)
    rospy.loginfo(f'Sent DELETEALL to {topic}')


def main():
    rospy.init_node('clear_env', anonymous=True)

    topic = rospy.get_param('~topic', '/env_marker')
    frame_id = rospy.get_param('~frame_id', 'world')
    object_id = rospy.get_param('~object_id', 'workspace_mesh')
    remove_all = rospy.get_param('~remove_all', False)

    clear_moveit_objects(object_id=object_id, remove_all=remove_all)
    clear_rviz_markers(topic=topic, frame_id=frame_id)

    rospy.loginfo('Environment clearing completed.')


if __name__ == '__main__':
    main()

