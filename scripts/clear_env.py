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


def clear_rviz_markers(topic: str, frame_id: str, object_id: str, remove_all: bool, duration_sec: float):
    pub = rospy.Publisher(topic, Marker, queue_size=1, latch=False)
    # wait a moment for connection
    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and (rospy.Time.now() - t0).to_sec() < 1.0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    if remove_all:
        m.action = Marker.DELETEALL
        action_desc = 'DELETEALL'
    else:
        # Delete a specific marker by ns/id pair (our publishers use id=0 and ns=object_id)
        m.ns = object_id
        m.id = 0
        m.type = Marker.MESH_RESOURCE  # match the published type for safety
        m.action = Marker.DELETE
        action_desc = f'DELETE ns={object_id} id=0'
    # Publish repeatedly for a short duration to win over any concurrent publishers
    end_time = rospy.Time.now() + rospy.Duration(max(0.0, duration_sec))
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    rospy.loginfo(f'Sent {action_desc} to {topic} for {duration_sec:.2f}s')


def main():
    rospy.init_node('clear_env', anonymous=True)

    topic = rospy.get_param('~topic', '/env_marker')
    frame_id = rospy.get_param('~frame_id', 'world')
    object_id = rospy.get_param('~object_id', 'workspace_mesh')
    clear_duration = float(rospy.get_param('~clear_duration_sec', 1.0))
    _remove_all_param = rospy.get_param('~remove_all', None)

    # New usage: use ~object_id:=all to remove all. Keep ~remove_all for backward compatibility.
    remove_all = bool(_remove_all_param) if _remove_all_param is not None else False
    if isinstance(object_id, str) and object_id.strip().lower() == 'all':
        remove_all = True
    if _remove_all_param not in (None, False):
        rospy.logwarn("~remove_all is deprecated; use ~object_id:=all instead.")

    clear_moveit_objects(object_id=object_id, remove_all=remove_all)
    clear_rviz_markers(topic=topic, frame_id=frame_id, object_id=object_id, remove_all=remove_all, duration_sec=clear_duration)

    rospy.loginfo('Environment clearing completed.')


if __name__ == '__main__':
    main()

