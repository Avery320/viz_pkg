#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from typing import Callable, Sequence, Optional, Tuple


class RvizMeshMarkerPublisher:
    """Periodic RViz mesh marker publisher.

    Publishes a visualization_msgs/Marker of type MESH_RESOURCE at a fixed rate.
    Pose is provided by a callable so the owner can control position/orientation.
    """

    def __init__(
        self,
        mesh_resource: str,
        frame_id: str = 'world',
        topic: str = '/env_marker',
        scale: Sequence[float] = (1.0, 1.0, 1.0),
        get_pose: Optional[Callable[[], Pose]] = None,
        color: Tuple[float, float, float, float] = (0.8, 0.8, 0.8, 0.4),
        use_embedded_materials: bool = False,
        period: float = 0.1,
        ns: str = 'mesh',
        marker_id: int = 0,
    ) -> None:
        self.mesh_resource = mesh_resource
        self.frame_id = frame_id
        self.scale = tuple(scale) if scale is not None else (1.0, 1.0, 1.0)
        self.get_pose = get_pose or self._default_pose
        self.color = color
        self.use_embedded_materials = use_embedded_materials
        self.ns = ns
        self.marker_id = int(marker_id)

        self._pub = rospy.Publisher(topic, Marker, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)

    def shutdown(self):
        try:
            self._timer.shutdown()
        except Exception:
            pass

    def _default_pose(self) -> Pose:
        p = Pose()
        p.orientation.w = 1.0
        return p

    def _on_timer(self, _evt):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.ns
        marker.id = self.marker_id
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_resource
        marker.pose = self.get_pose()
        marker.scale.x, marker.scale.y, marker.scale.z = self.scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.color
        marker.mesh_use_embedded_materials = self.use_embedded_materials
        self._pub.publish(marker)

