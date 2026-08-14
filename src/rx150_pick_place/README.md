# rx150_pick_place — Layer 2: QUYẾT ĐỊNH

Node đọc **nhận diện (Layer 1)** + **cử chỉ tay** → chọn vật → gọi **MoveIt (Layer 3)** gắp & thả.
Toàn bộ chuyển động đi qua `move_group` (action `move_action`); KHÔNG bao giờ publish trực tiếp
`/rx150/commands/*` (sẽ đánh `fuzzy_node`). SDK Interbotix chỉ dùng làm **IK-oracle**
(`set_ee_pose_components(execute=False)` — tính góc khớp, không gửi lệnh).

## Sơ đồ pipeline 3 layer

```
D435 ──rs_launch──► /camera/camera/{color, aligned_depth_to_color, depth/color/points}
                      │                                      │
        ┌─────────────┘                                      └──► OctoMap (sensors_3d.yaml)
        ▼                                                          ──► move_group collision
[L1 rx150_perception] yolo_detector_node ──► /yolo/detected_objects (PoseArray base_link)
                      hand_gesture_node  ──► /hand_gesture/selected_target (Int32)  ← "chỉ tay"
                                           ──► /hand_gesture/event "ok_sign"        ← xác nhận gắp
        ▼
[L2 rx150_pick_place]  pick_place_moveit_node  ← node này (bộ não)
        │   IK-oracle (execute=False) → joint goal
        │   gửi /move_action (interbotix_arm)   ──► move_group OMPL ──► arm_bridge ──► fuzzy_node ──► xs_sdk
        │   gửi gripper goal (interbotix_gripper) ──► gripper_bridge ──► xs_sdk
        │   startup: add box bàn qua /apply_planning_scene
[L3 fuzzy_controller + MoveIt]  motion (hoạch định + thực thi PWM)
```

| Layer | Package | Vai trò |
|---|---|---|
| L1 Nhận diện | `rx150_perception` | YOLO seg + MediaPipe → pose vật + chọn vật bằng tay |
| **L2 Quyết định** | **`rx150_pick_place`** | **chọn grasp, place, gọi MoveIt, add box bàn** |
| L3 Motion | `fuzzy_controller` + MoveIt config | `move_group` OMPL + 2 bridge + `fuzzy_node` PWM |

## Cách chạy (2 terminal, đã `source ~/interbotix_ws/source_all.sh`)

**T1 — motion stack + camera + hand-eye:**
```bash
ros2 launch fuzzy_controller fuzzy_moveit.launch.py \
    use_camera:=true rs_camera_pointcloud_enable:=true \
    use_camera_static_tf:=false use_handeye_publisher:=true
```

**T2 — perception + quyết định (cái này):**
```bash
ros2 launch rx150_pick_place pick_place.launch.py
```

Sau đó: **đưa tay vào camera, "chỉ tay" vào vật muốn gắp → làm OK-sign** → robot gắp vật đó
rồi thả ở place zone (cấu hình trong `config/pick_place_params.yaml`).

> ⚠️ **An toàn trước T1:** `pkill -f xs_sdk` (2 driver trên 1 bus → crash); cánh tay thoáng
> (fuzzy_node PWM-drive về home ngay khi launch); e-stop trong tầm tay.

## Kiểm tra từng layer (debug)

| Layer | Lệnh kiểm tra |
|---|---|
| Camera | `ros2 topic hz /camera/camera/depth/color/points` ≈ 30Hz |
| L1 detection | `ros2 topic echo /yolo/detected_objects` (xyz base-link hợp lý) |
| L1 gesture | `ros2 topic echo /hand_gesture/selected_target` + `/hand_gesture/event` |
| L2 MoveIt | xem log `pick_place_moveit`: `==== PICK object #i tại ... ====` + mỗi bước APPROACH/DESCEND/... |
| L3 bridge | log `gripper_trajectory_bridge` (Grasp SUCCESS) + `fuzzy_trajectory_bridge` |

## State machine (mỗi bước = 1 goal MoveGroup)
```
IDLE --ok_sign--> APPROACH(pre) -> DESCEND(grasp) -> GRASP -> LIFT(pre)
     -> TRANSPORT(pre-place) -> PLACE -> RELEASE -> RETREAT -> HOME -> IDLE
```
Bất kỳ bước nào fail (IK hoặc MoveIt error ∉ {1,-4}) → **abort**: mở gripper + về home.

## Tham số (`config/pick_place_params.yaml`) — TUNE cho bàn thật
- `approach_delta` (0.05m): chênh z tiếp cận/rút.
- `grasp_pitch` (0.5rad): nghiêng tay gắp — **DOF định hướng duy nhất** trên 5-DoF.
- `finger_grasp_offset` (0.02m): hạ thêm để ngón kẹp hai bên vật.
- `place_x/y/z/pitch`: vị trí thả (TUNE).
- `home_joints`: pose nghỉ an toàn.
- `use_gripper_bridge` (true): gripper qua MoveIt bridge; **false** = raw PWM fallback.
- `add_table_collision` (true): thêm box bàn tránh đâm; `table_*` = tâm & kích thước box.

### 5-DoF — lưu ý quan trọng
rx150 có 5 khớp → chỉ đạt **position + pitch**. SDK ép `yaw = atan2(y,x)` (waist quay về phía vật).
Yaw do detector ước lượng **không dùng được** làm tool rotation → node không truyền yaw.

## Gripper: bridge vs PWM fallback
- **Bridge (mặc định, `use_gripper_bridge: true`):** gửi goal nhóm `interbotix_gripper`
  (joint `left_finger`, 0.015=grasp / 0.037=release) qua `/rx150/gripper_controller/...`.
  Bridge stall-aware (chạm vật = SUCCESS) + giữ lực khi vận chuyển. **Khuyến nghị.**
- **PWM fallback (`use_gripper_bridge: false`):** `JointSingleCommand(name='gripper')`
  PWM ±200. Dùng nếu path MoveGroup-gripper gặp vấn đề mimic-joint/controller.

## Collision
- **Box bàn:** node gọi `/apply_planning_scene` (ADD 1 `CollisionObject` box ở `world`) khi pick đầu.
  TUNE `table_z`/`table_size_z` để **đỉnh box ≤ mặt bàn**, và grasp z giữ ngón TRÊN box — nếu không
  grasp pose self-collision → mọi plan fail.
- **OctoMap:** từ `/camera/camera/depth/color/points` (`rx150_perception/config/sensors_3d.yaml`),
  nạp vào `move_group`. Cần camera publish pointcloud (T1).

## Build
```bash
cd ~/interbotix_ws && colcon build --packages-select rx150_pick_place
source install/setup.bash
```
