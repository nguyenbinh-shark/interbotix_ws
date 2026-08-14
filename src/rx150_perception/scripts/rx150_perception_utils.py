#!/usr/bin/env python3
"""
rx150_perception_utils — Utility functions dùng chung cho YOLO và hand-gesture.

Gồm 5 hàm chia sẻ trước đây:
- deproject: back-projection pinhole
- to_base_point/to_base_array: transform point TF đến base frame
- robust_depth_m: median depth trong patch
- make_camera_sync: tạo color+depth ApproximateTimeSynchronizer
- CameraInfo caching: _info_cb (phần này implement riêng trong mỗi node)

Cách dùng (trong node):
  from rx150_perception_utils import deproject, to_base_array, robust_depth_m, make_camera_sync
"""

import numpy as np
import message_filters
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


def deproject(u, v, z_m, intrinsics):
    """
    Back-projection pinhole: pixel (u,v) + độ sâu z_m → điểm 3D (x,y,z) trong optical frame.

    Args:
        u, v: tọa độ pixel
        z_m: độ sâu (mét)
        intrinsics: CameraInfo (có k, cx, cy)

    Returns:
        (x, y, z) tuple trong optical frame
    """
    fx = intrinsics.k[0]  # focal length x
    fy = intrinsics.k[4]  # focal length y
    cx = intrinsics.k[2]  # principal point x
    cy = intrinsics.k[5]  # principal point y
    return (u - cx) * z_m / fx, (v - cy) * z_m / fy, z_m


def to_base_point(x, y, z, stamp, tf, base_frame, optical_frame):
    """
    Transform point (x,y,z) từ optical frame sang base frame qua TF.

    Args:
        x, y, z: point trong optical frame
        stamp: timestamp (header.stamp)
        tf: tf2_ros.Buffer
        base_frame, optical_frame: frame IDs

    Returns:
        PointStamped trong base frame (hoặc None nếu TF không available)
    """
    ps = PointStamped()
    ps.header.frame_id = optical_frame
    ps.header.stamp = stamp
    ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
    try:
        return do_transform_point(ps, tf)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
        return None


def to_base_array(points_array, stamp, tf, base_frame, optical_frame):
    """
    Transform array nhiều điểm (N×3) từ optical sang base frame.

    Args:
        points_array: numpy array (N, 3) — mỗi hàng là (x,y,z)
        stamp, tf, base_frame, optical_frame: như trên

    Returns:
        numpy array (N, 3) trong base frame, hoặc None nếu lỗi
    """
    n = points_array.shape[0]
    result = np.zeros((n, 3))
    for i in range(n):
        transformed = to_base_point(
            points_array[i, 0], points_array[i, 1], points_array[i, 2],
            stamp, tf, base_frame, optical_frame
        )
        if transformed is None:
            return None
        result[i] = [transformed.point.x, transformed.point.y, transformed.point.z]
    return result


def robust_depth_m(depth_mm, cx, cy, patch=5):
    """
    Median depth (mét) trong patch NxN quanh (cx,cy), bỏ pixel invalid (=0).

    Args:
        depth_mm: depth image (mm) từ aligned_depth
        cx, cy: center pixel
        patch: kích thước patch (mặc định 5×5)

    Returns:
        float: độ sâu trung vị (mét)
    """
    h, w = depth_mm.shape
    half = patch // 2
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    patch_region = depth_mm[y0:y1, x0:x1]
    valid = patch_region[patch_region > 0]
    return float(np.median(valid)) / 1000.0 if valid.size else 0.0


def make_camera_sync(node, image_topic, depth_topic, slop=0.1):
    """
    Tạo ApproximateTimeSynchronizer cho color + depth.

    Args:
        node: rclpy node
        image_topic, depth_topic: topic names
        slop: max time diff (giây)

    Returns:
        ApproximateTimeSynchronizer đã registerCallback
    """
    color_sub = message_filters.Subscriber(node, image_topic)
    depth_sub = message_filters.Subscriber(node, depth_topic)
    sync = message_filters.ApproximateTimeSynchronizer(
        [color_sub, depth_sub], 10, slop)
    return sync, color_sub, depth_sub
