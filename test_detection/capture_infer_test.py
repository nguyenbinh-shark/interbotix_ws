#!/usr/bin/env python3
"""Test model nhận diện OFFLINE: capture frame thật từ RealSense D435, chạy YOLO seg,
tái lập đúng code path của yolo_detector_node.py (mask-centroid, robust depth, deproject,
cap-orange HSV, yaw). KHÔNG cần ROS/TF — tính 3D trong camera frame, yaw camera-frame."""
import json, math, os, sys, time
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

OUT = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = '/home/hust/interbotix_ws/src/rx150_perception/models/best_color.pt'


def robust_depth_m(depth_mm, cx, cy, patch=11):
    h, w = depth_mm.shape
    half = patch // 2
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    p = depth_mm[y0:y1, x0:x1]
    v = p[p > 0]
    return float(np.median(v)) / 1000.0 if v.size else 0.0


def cap_pixel(color_bgr, bin_mask):
    """Port _cap_pixel: centroid nắp cam (HSV inRange 5-20). Trả (x,y,area) hoặc None."""
    ys, xs = np.where(bin_mask > 0)
    if xs.size == 0:
        return None
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    roi = color_bgr[y0:y1 + 1, x0:x1 + 1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    cap = cv2.inRange(hsv, (5, 100, 100), (20, 255, 255))
    cnts, _ = cv2.findContours(cap, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    m = cv2.moments(c)
    if m['m00'] == 0:
        return None
    return (x0 + int(m['m10'] / m['m00']), y0 + int(m['m01'] / m['m00']),
            cv2.contourArea(c))


def deproject(u, v, z, fx, fy, px, py):
    return ((u - px) * z / fx, (v - py) * z / fy, z)


# ---------------- capture ----------------
ctx = rs.context()
dev = ctx.query_devices()[0]
# bật emitter + auto-exposure để depth/color có dữ liệu
for s in dev.query_sensors():
    try:
        if s.is_depth_sensor():
            s.set_option(rs.option.emitter_enabled, 1)
            s.set_option(rs.option.enable_auto_exposure, 1)
        else:
            s.set_option(rs.option.enable_auto_exposure, 1)
    except Exception as e:
        print('sensor option warn:', e)

pipeline = rs.pipeline(ctx)
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
try:
    profile = pipeline.start(cfg)
except RuntimeError as e:
    print(f'\n!! pipeline.start thất bại: {e}\n'
          '   Camera đang bị process khác giữ (rs_launch / realsense-viewer /\n'
          '   capture_infer_test cũ). Tìm + kill:\n'
          '     ps -eo pid,cmd | grep -iE "realsense|capture_infer" | grep -v grep\n'
          '   KHÔNG hardware_reset (làm re-enumerate /dev/video → lỗi node).')
    sys.exit(1)
align = rs.align(rs.stream.color)
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fx, fy, px, py = intr.fx, intr.fy, intr.ppx, intr.ppy
print(f'INTRINSICS fx={fx:.2f} fy={fy:.2f} ppx={px:.2f} ppy={py:.2f} {intr.width}x{intr.height}')
# warmup: retry lấy frame, timeout dài, discard 10 frame đầu cho AE/emitter ổn định
frames = []
got = 0
for attempt in range(80):
    try:
        fs = pipeline.wait_for_frames(timeout_ms=15000)
    except RuntimeError as e:
        print(f'wait_for_frames retry {attempt}: {e}')
        time.sleep(0.5)
        continue
    fs = align.process(fs)
    c, d = fs.get_color_frame(), fs.get_depth_frame()
    if not (c and d):
        continue
    got += 1
    if got <= 10:
        continue  # discard warmup
    frames.append((np.asanyarray(c.get_data()), np.asanyarray(d.get_data())))
    if len(frames) >= 20:
        break
pipeline.stop()
assert frames, 'KHÔNG lấy được frame nào từ D435!'
color, depth = frames[-1]
cv2.imwrite(f'{OUT}/frame_color.png', color)
cv2.imwrite(f'{OUT}/frame_depth.png', depth)
json.dump({'fx': fx, 'fy': fy, 'ppx': px, 'ppy': py,
           'w': intr.width, 'h': intr.height}, open(f'{OUT}/camera_info.json', 'w'), indent=2)
nz = 100.0 * np.count_nonzero(depth) / depth.size
dmed = float(np.median(depth[depth > 0])) / 1000.0 if np.any(depth > 0) else 0.0
print(f'captured {len(frames)} frames. depth nonzero={nz:.1f}%  median(valid)={dmed:.3f}m')

# ---------------- inference ----------------
model = YOLO(WEIGHTS)
print('CLASSES:', model.names)
h, w = color.shape[:2]
summary = {'classes': model.names, 'frames_captured': len(frames),
           'depth_nonzero_pct': round(nz, 1), 'depth_median_m': round(dmed, 3),
           'results': []}
for conf in [0.5, 0.25]:
    res = model.predict(source=color, conf=conf, verbose=False)[0]
    ann = res.plot()
    masks = res.masks.data.cpu().numpy() if (getattr(res, 'masks', None) is not None) else None
    dets = []
    for i, box in enumerate(res.boxes):
        cls = model.names[int(box.cls[0])]
        cf = float(box.conf[0])
        cx = cy = None
        bin_mask = area = None
        if masks is not None and i < len(masks):
            m = cv2.resize(masks[i], (w, h), cv2.INTER_NEAREST)
            bin_mask = (m > 0.5).astype(np.uint8)
            cnts, _ = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                mm = cv2.moments(max(cnts, key=cv2.contourArea))
                if mm['m00'] > 0:
                    cx, cy = int(mm['m10'] / mm['m00']), int(mm['m01'] / mm['m00'])
                area = int(sum(cv2.contourArea(c) for c in cnts))
        if cx is None:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        z = robust_depth_m(depth, cx, cy)
        xc, yc, zc = deproject(cx, cy, z, fx, fy, px, py)
        yaw_deg = cap = None
        if bin_mask is not None:
            cp = cap_pixel(color, bin_mask)
            if cp:
                z2 = robust_depth_m(depth, cp[0], cp[1])
                if z2 > 0:
                    xc2, yc2, _ = deproject(cp[0], cp[1], z2, fx, fy, px, py)
                    yaw_deg = round(-math.degrees(math.atan2(yc2 - yc, xc2 - xc)), 1)
                    cap = cp[:2]
                    cv2.circle(ann, (cp[0], cp[1]), 4, (0, 0, 255), -1)  # đỏ = nắp cam
        cv2.circle(ann, (cx, cy), 4, (0, 255, 0), -1)  # xanh lá = centroid
        cv2.putText(ann, f'{cls} z={z:.3f} {"yaw=" + str(yaw_deg) if yaw_deg is not None else ""}',
                    (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        dets.append({'class': cls, 'conf': round(cf, 3), 'cx': cx, 'cy': cy,
                     'z_m': round(z, 3), 'xyz_cam_m': [round(xc, 3), round(yc, 3), round(zc, 3)],
                     'mask_area_px': area, 'yaw_deg': yaw_deg, 'cap_px': cap})
    cv2.imwrite(f'{OUT}/annotated_conf{conf}.png', ann)
    summary['results'].append({'conf': conf, 'n_det': len(dets), 'detections': dets})
    print(f'\n===== conf={conf}: {len(dets)} detection(s) =====')
    for d in dets:
        print(f"  {d['class']:<8} conf={d['conf']:.2f} "
              f"px=({d['cx']},{d['cy']}) z={d['z_m']:.3f}m "
              f"area={d['mask_area_px']} yaw={d['yaw_deg']} cap={d['cap_px']}")
        print(f"           xyz_cam(m)=({d['xyz_cam_m'][0]:.3f},{d['xyz_cam_m'][1]:.3f},{d['xyz_cam_m'][2]:.3f})")

json.dump(summary, open(f'{OUT}/summary.json', 'w'), indent=2, ensure_ascii=False)
print('\nDONE →', OUT)
