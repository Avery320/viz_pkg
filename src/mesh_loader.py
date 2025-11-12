#!/usr/bin/env python3

"""
mesh_loader.py — Utilities to load STL triangles for visualization.

Tries loaders in the following order:
  1) trimesh (best compatibility and performance)
  2) numpy-stl
  3) simple ASCII STL parser (fallback; binary not supported)

Returns flattened list of vertices (3 points per triangle), optionally as
geometry_msgs/Point instances to be used directly by TRIANGLE_LIST markers.
"""
from typing import Iterable, List, Sequence, Tuple


def _get_loggers():
    try:
        import rospy  # type: ignore
        return rospy.loginfo, rospy.logwarn, rospy.logerr
    except Exception:
        def _info(msg: str):
            print("[INFO] ", msg)
        def _warn(msg: str):
            print("[WARN] ", msg)
        def _err(msg: str):
            print("[ERROR]", msg)
        return _info, _warn, _err


def _to_points(raw_xyz: Iterable[Tuple[float, float, float]]):
    # Lazy import to avoid ROS dependency when only numeric output is needed
    from geometry_msgs.msg import Point  # type: ignore
    pts: List[Point] = []
    for x, y, z in raw_xyz:
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        pts.append(p)
    return pts


def load_stl_triangles(
    abs_path: str,
    scale: Sequence[float] = (1.0, 1.0, 1.0),
    as_points: bool = False,
):
    """Load triangles from an STL file and return flattened vertex list.

    Args:
        abs_path: Absolute filesystem path to an STL file.
        scale: (sx, sy, sz) per-axis scaling.
        as_points: If True, returns List[geometry_msgs.msg.Point]; otherwise
            returns List[Tuple[float, float, float]].

    Returns:
        List of vertices flattened (3 per triangle). The list may be empty on
        failure.
    """
    logi, logw, loge = _get_loggers()
    sx, sy, sz = (scale or (1.0, 1.0, 1.0))

    # Try trimesh first
    try:
        import trimesh  # type: ignore
        m = trimesh.load(abs_path, force='mesh')
        if hasattr(m, 'triangles'):
            raw: List[Tuple[float, float, float]] = []
            for tri in m.triangles:  # (N,3,3)
                for v in tri:
                    raw.append((float(v[0]) * sx, float(v[1]) * sy, float(v[2]) * sz))
            logi(f"Triangles loaded via trimesh: {len(raw)//3} faces")
            return _to_points(raw) if as_points else raw
    except Exception:
        pass

    # Fallback: numpy-stl
    try:
        from stl import mesh as stl_mesh  # type: ignore
        m = stl_mesh.Mesh.from_file(abs_path)
        raw2: List[Tuple[float, float, float]] = []
        for tri in m.vectors:  # (N,3,3)
            for v in tri:
                raw2.append((float(v[0]) * sx, float(v[1]) * sy, float(v[2]) * sz))
        logi(f"Triangles loaded via numpy-stl: {len(raw2)//3} faces")
        return _to_points(raw2) if as_points else raw2
    except Exception:
        pass

    # Last resort: very simple ASCII STL parser
    try:
        cnt = 0
        raw3: List[Tuple[float, float, float]] = []
        with open(abs_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                ls = line.strip().split()
                if len(ls) >= 4 and ls[0].lower() == 'vertex':
                    x, y, z = float(ls[1]), float(ls[2]), float(ls[3])
                    raw3.append((x * sx, y * sy, z * sz))
                    cnt += 1
        if cnt % 3 != 0:
            logw('ASCII STL parse produced non-multiple-of-3 vertices; rendering may be incorrect.')
        logi(f"Triangles loaded via ASCII parse: {cnt//3} faces")
        return _to_points(raw3) if as_points else raw3
    except Exception as e:
        loge(f"Failed to load triangles from STL: {e}")
        return []

