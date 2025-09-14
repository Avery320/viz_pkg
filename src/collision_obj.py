#!/usr/bin/env python3

import sys
from typing import Callable, Sequence, Tuple
import rospy
from geometry_msgs.msg import Pose, PoseStamped

# Optional MoveIt integration
try:
    import moveit_commander
    from moveit_commander import PlanningSceneInterface
    MOVEIT_AVAILABLE = True
except Exception as e:
    MOVEIT_AVAILABLE = False
    _MOVEIT_IMPORT_ERROR = e


class CollisionObjectPublisher:
    """Add/remove a mesh as a collision object in MoveIt PlanningSceneInterface.

    get_pose: a callable returning geometry_msgs/Pose for current placement.
    scale: (sx, sy, sz)
    """

    def __init__(self,
                 object_id: str,
                 frame_id: str,
                 get_pose: Callable[[], Pose],
                 mesh_abs_path: str,
                 scale: Sequence[float] = (1.0, 1.0, 1.0),
                 add_delay: float = 1.0):
        self.object_id = object_id
        self.frame_id = frame_id
        self.get_pose = get_pose
        self.mesh_abs_path = mesh_abs_path
        self.scale = tuple(scale)
        self.scene = None

        if MOVEIT_AVAILABLE:
            try:
                moveit_commander.roscpp_initialize(sys.argv)
                self.scene = PlanningSceneInterface(synchronous=True)
                # add after a short delay to allow scene to be ready
                rospy.Timer(rospy.Duration(add_delay), self._add_once, oneshot=True)
            except Exception as e:
                rospy.logerr(f'Failed to initialize MoveIt: {e}')
        else:
            rospy.logwarn(f'moveit_commander not available; collision object will not be added (error: {_MOVEIT_IMPORT_ERROR})')

    def _add_once(self, _event):
        if not self.scene:
            return
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.pose = self.get_pose()
        try:
            self.scene.add_mesh(self.object_id, ps, self.mesh_abs_path, size=self.scale)
            rospy.loginfo(f'Added collision mesh: id={self.object_id}, file={self.mesh_abs_path}')
        except Exception as e:
            rospy.logerr(f'Failed to add collision mesh: {e}')

    def remove(self):
        if not self.scene:
            return
        try:
            self.scene.remove_world_object(self.object_id)
            rospy.loginfo(f'Removed collision mesh: id={self.object_id}')
        except Exception as e:
            rospy.logerr(f'Failed to remove collision mesh: {e}')

