#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
from geometry_msgs.msg import Pose
from typing import List, Set


# Import separated RViz marker module (flat module)
from rviz_marker import RvizMeshMarkerPublisher

# Collision object manager (separated module)
from collision_obj import CollisionObjectPublisher


class ObjPublisher:
    """Load STL mesh, publish RViz marker, and add collision via CollisionObjectPublisher."""

    def __init__(self):
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.object_id = rospy.get_param('~object_id', 'workspace_mesh')
        self.scale_xyz = rospy.get_param('~scale', [0.001, 0.001, 0.001])
        self.pos_xyz = self._parse_xyz_from_argv(default=(0.0, 0.0, 0.0))

        # Resolve mesh resource paths (STL only), support multiple
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('viz_pkg')

        raw_paths = rospy.get_param('~mesh_paths', None)
        if raw_paths is None:
            # fallback to single ~mesh_path; auto-detect default location (config/ or modol/)
            single = rospy.get_param('~mesh_path', None)
            if single is None:
                candidates = ['config/3dcp_ws.stl', 'modol/3dcp_ws.stl', 'modol/module.stl']
                chosen = None
                for c in candidates:
                    if os.path.exists(os.path.join(self.package_path, c)):
                        chosen = c
                        break
                if chosen is None:
                    rospy.logwarn('No default STL found under config/ or modol/. Please set ~mesh_path or ~mesh_paths.')
                    chosen = 'config/3dcp_ws.stl'
                single = chosen
            paths: List[str] = [single]
        else:
            if isinstance(raw_paths, str):
                # allow comma-separated string
                paths = [p.strip() for p in raw_paths.split(',') if p.strip()]
            else:
                paths = list(raw_paths)

        def _rel_from_param(p: str) -> str:
            if p.startswith('package://viz_pkg/'):
                return p[len('package://viz_pkg/'):]
            return p

        used: Set[str] = set()
        items = []
        for p in paths:
            rel = _rel_from_param(p)
            if not rel.lower().endswith('.stl'):
                rospy.logwarn(f"mesh_paths contains non-.stl entry '{rel}'. It will still be attempted but may fail.")
            abs_path = os.path.join(self.package_path, rel)
            res_uri = 'package://viz_pkg/' + rel
            base = os.path.splitext(os.path.basename(rel))[0]
            # sanitize id: lowercase, alnum and '_' only
            candidate = ''.join(ch if (ch.isalnum() or ch == '_') else '_' for ch in base).lower().strip('_') or 'obj'
            obj_id = candidate
            i = 2
            while obj_id in used:
                obj_id = f"{candidate}_{i}"
                i += 1
            used.add(obj_id)
            items.append({'id': obj_id, 'rel': rel, 'abs': abs_path, 'res': res_uri})

        self._items = items

        # RViz markers (per object), periodic publishing
        marker_color = rospy.get_param('~marker_color', [0.8, 0.8, 0.8, 0.4])
        if not (isinstance(marker_color, (list, tuple)) and len(marker_color) == 4):
            rospy.logwarn('~marker_color must be [r,g,b,a]; using default (0.8,0.8,0.8,0.4)')
            marker_color = [0.8, 0.8, 0.8, 0.4]
        use_embedded_mats = rospy.get_param('~marker_use_embedded_materials', False)
        marker_period = float(rospy.get_param('~marker_period', 0.1))

        self._markers: List[RvizMeshMarkerPublisher] = []
        for it in self._items:
            self._markers.append(
                RvizMeshMarkerPublisher(
                    mesh_resource=it['res'],
                    frame_id=self.frame_id,
                    topic='/env_marker',
                    scale=self.scale_xyz,
                    get_pose=self._make_pose,
                    color=tuple(marker_color),
                    use_embedded_materials=bool(use_embedded_mats),
                    period=marker_period,
                    ns=it['id'],  # avoid overwrite in same topic
                    marker_id=0,
                )
            )

        # MoveIt collision objects (per object)
        self._collisions: List[CollisionObjectPublisher] = []
        for it in self._items:
            self._collisions.append(
                CollisionObjectPublisher(
                    object_id=it['id'],
                    frame_id=self.frame_id,
                    get_pose=self._make_pose,
                    mesh_abs_path=it['abs'],
                    scale=tuple(self.scale_xyz),
                    add_delay=1.0,
                )
            )

        self._log_startup()

    def _parse_xyz_from_argv(self, default=(0.0, 0.0, 0.0)):
        if len(sys.argv) >= 4:
            try:
                return float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
            except Exception:
                rospy.logwarn('Invalid CLI args; using default position (x y z)')
        return default

    def _make_pose(self) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.pos_xyz
        pose.orientation.w = 1.0
        return pose


    def _log_startup(self):
        rospy.loginfo('ObjPublisher started (multi-STL)')
        rospy.loginfo(f'frame_id: {self.frame_id}')
        rospy.loginfo(f'pos: {self.pos_xyz}, scale: {self.scale_xyz}')
        for it in self._items:
            rospy.loginfo(f"object_id: {it['id']}  mesh: {it['rel']}")
        any_scene = any(getattr(c, 'scene', None) is not None for c in getattr(self, '_collisions', []))
        if any_scene:
            rospy.loginfo('Will add meshes to MoveIt planning scene (collision objects)')
        else:
            rospy.logwarn('MoveIt planning scene disabled; RViz visualization only')
