#!/usr/bin/env python3
# yolo_detector_node.py — YOLOv8 segmentation → PoseArray (base frame) + roll/yaw.
#
# Harvest từ key_point/task3_4tubes.py: mask-centroid (`cv2.moments`), roll-from-cap
# (HSV→atan2 trong base frame), EMA smoothing. BỎ pyrealsense2 + ma trận M02 hardcode;
# dùng topic realsense2_camera (color + aligned_depth + CameraInfo) và tf2 hand-eye
# (camera_color_optical_frame → rx150/base_link).
#
# Topic ra:
#   ~/detected_objects  (geometry_msgs/PoseArray, base frame, yaw trong orientation)
#   ~/class_names       (std_msgs/StringMultiArray, index-aligned với PoseArray)
#   ~/debug/image_raw   (sensor_msgs/Image, mask + centroid để tune)

import os
import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from std_msgs.msg import StringMultiArray
from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf2_geometry_msgs

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover
    try:
        from ament_index_python.packages import get_package_share_directory
        req_path = os.path.join(get_package_share_directory('rx150_perception'), 'requirements.txt')
        raise ImportError(
            f'ultralytics chưa cài. Chạy: pip install --user -r {req_path}'
        ) from exc
    except ImportError:
        # Fallback nếu ament_index_python không available (có thể xảy ra khi chạy trong test)
        raise ImportError(
            'ultralytics chưa cài. Chạy: pip install --user -r src/rx150_perception/requirements.txt'
        ) from exc


def ema(prev, new, alpha):
    """EMA low-pass — port ema_filter/ema_filter_roll của key_point."""
    return new if prev is None else alpha * new + (1.0 - alpha) * prev


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # --- params ---
        self.declare_parameter('weights_path', '')
        self.declare_parameter('base_frame', 'rx150/base_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('depth_patch_size', 5)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('enable_roll', True)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')

        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('camera_optical_frame').value
        self.conf = float(self.get_parameter('confidence').value)
        self.patch = int(self.get_parameter('depth_patch_size').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)
        self.enable_roll = bool(self.get_parameter('enable_roll').value)

        weights = self.get_parameter('weights_path').value
        if not weights:
            from ament_index_python.packages import get_package_share_directory
            weights = os.path.join(
                get_package_share_directory('rx150_perception'), 'models', 'best_color.pt')
        self.get_logger().info(f'Nạp weights: {weights}')
        self.model = YOLO(weights)
        self.get_logger().info(f'Classes: {self.model.names}')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.intrinsics = None
        self._rolls = {}  # class name -> EMA yaw (base frame)

        # --- pubs ---
        self.pose_pub = self.create_publisher(PoseArray, '/yolo/detected_objects', 10)
        self.cls_pub = self.create_publisher(StringMultiArray, '/yolo/class_names', 10)

        # Debug image: gate sau param publish_debug_image (mặc định false để giảm cost)
        self.declare_parameter('publish_debug_image', False)
        self._publish_debug = self.get_parameter('publish_debug_image').value
        if self._publish_debug:
            self.img_pub = self.create_publisher(Image, '/yolo/debug/image_raw', 10)
        else:
            self.img_pub = None

        # --- subs ---
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._info_cb, 10)
        color_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('image_topic').value)
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 10, 0.1)
        self.sync.registerCallback(self._sync_cb)
        self.get_logger().info('yolo_detector sẵn sàng.')

    # ------------------------------------------------------------------ #
    def _info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg
            self.get_logger().info('Đã nhận CameraInfo (intrinsics).')

    def _robust_depth_m(self, depth_mm, cx, cy):
        """Median depth (mét) trong patch NxN quanh (cx,cy), bỏ pixel invalid (=0).
        Tương đương get_avg_depth của key_point + get_robust_depth của vision_node."""
        h, w = depth_mm.shape
        half = self.patch // 2
        y0, y1 = max(0, cy - half), min(h, cy + half + 1)
        x0, x1 = max(0, cx - half), min(w, cx + half + 1)
        patch = depth_mm[y0:y1, x0:x1]
        valid = patch[patch > 0]
        return float(np.median(valid)) / 1000.0 if valid.size else 0.0

    def _cap_pixel(self, color_bgr, bin_mask, cx, cy):
        """Tìm centroid pixel của nắp cam (HSV inRange) — port task3_4tubes.py:644-658.
        Trả (capx, capy) hoặc None."""
        ys, xs = np.where(bin_mask > 0)
        if xs.size == 0:
            return None
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        roi = color_bgr[y0:y1 + 1, x0:x1 + 1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        cap_mask = cv2.inRange(hsv, (5, 100, 100), (20, 255, 255))
        cnts, _ = cv2.findContours(cap_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        m = cv2.moments(c)
        if m['m00'] == 0:
            return None
        return x0 + int(m['m10'] / m['m00']), y0 + int(m['m01'] / m['m00'])

    def _deproject(self, u, v, z_m, fx, fy, px, py):
        """Back-projection pinhole — đẳng thức với rs2_deproject_pixel_to_point."""
        return (u - px) * z_m / fx, (v - py) * z_m / fy, z_m

    def _to_base(self, x, y, z, stamp, tf):
        ps = PointStamped()
        ps.header.frame_id = self.optical_frame
        ps.header.stamp = stamp
        ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
        return tf2_geometry_msgs.do_transform_point(ps, tf)

    # ------------------------------------------------------------------ #
    def _sync_cb(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        # TF camera_optical → base (một lần/frame, dùng cho mọi object + cap).
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'Chưa có TF {self.base_frame}<-{self.optical_frame}: {e}',
                throttle_duration_sec=3.0)
            return

        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'imgmsg→cv2 lỗi: {e}', throttle_duration_sec=2.0)
            return

        try:
            res = self.model.predict(source=color, conf=self.conf, verbose=False)[0]
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'YOLO infer lỗi: {e}', throttle_duration_sec=2.0)
            return
        annotated = res.plot()

        k = self.intrinsics.k
        fx, fy, px, py = k[0], k[4], k[2], k[5]
        stamp = color_msg.header.stamp
        h, w = color.shape[:2]

        masks = None
        if getattr(res, 'masks', None) is not None and getattr(res.masks, 'data', None) is not None:
            masks = res.masks.data.cpu().numpy()

        poses, classes = [], []
        for i, box in enumerate(res.boxes):
            cls_id = int(box.cls[0])
            cname = self.model.names.get(cls_id, str(cls_id))

            # centroid: ưu tiên mask-centroid (seg), fallback bbox-center (detect).
            cx = cy = None
            bin_mask = None
            if masks is not None and i < len(masks):
                m = cv2.resize(masks[i], (w, h), interpolation=cv2.INTER_NEAREST)
                bin_mask = (m > 0.5).astype(np.uint8)
                cnts, _ = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    mm = cv2.moments(max(cnts, key=cv2.contourArea))
                    if mm['m00'] > 0:
                        cx = int(mm['m10'] / mm['m00'])
                        cy = int(mm['m01'] / mm['m00'])
            if cx is None:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            z = self._robust_depth_m(depth_mm, cx, cy)
            if z <= 0:
                continue
            xc, yc, zc = self._deproject(cx, cy, z, fx, fy, px, py)
            center_b = self._to_base(xc, yc, zc, stamp, tf)

            # yaw trong base frame từ hướng center→cap (port task3_4tubes.py:661-670).
            yaw = 0.0
            if self.enable_roll and bin_mask is not None:
                cap = self._cap_pixel(color, bin_mask, cx, cy)
                if cap is not None:
                    zc2 = self._robust_depth_m(depth_mm, cap[0], cap[1])
                    if zc2 > 0:
                        xc2, yc2, _ = self._deproject(cap[0], cap[1], zc2, fx, fy, px, py)
                        cap_b = self._to_base(xc2, yc2, zc2, stamp, tf)
                        dx = cap_b.point.x - center_b.point.x
                        dy = cap_b.point.y - center_b.point.y
                        raw_yaw = -math.atan2(dy, dx)  # dấu theo key_point
                        yaw = ema(self._rolls.get(cname), raw_yaw, self.alpha)
                        self._rolls[cname] = yaw

            pose = Pose()
            pose.position = center_b.point
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
            classes.append(cname)
            cv2.circle(annotated, (cx, cy), 4, (0, 255, 0), -1)
            cv2.putText(annotated, f'{cname}', (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if poses:
            pa = PoseArray()
            pa.header.frame_id = self.base_frame
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.poses = poses
            self.pose_pub.publish(pa)
            self.cls_pub.publish(StringMultiArray(data=classes))

        # Publish debug image (nếu được bật) — cv2_to_imgmsg cost cao
        if self._publish_debug and self.img_pub is not None:
            try:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
