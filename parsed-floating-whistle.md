# Plan: Điều khiển rx150 qua MoveIt bằng fuzzy controller (bridge node)

## Context
Hiện có hai đường điều khiển **loại nhau**:
- **MoveIt** (`xsarm_moveit`): `move_group` → ros2_control `JointTrajectoryController` (position mode) → driver, dùng `interbotix_xsarm_ros_control`.
- **Fuzzy** (`fuzzy_control`): `fuzzy_node` (chuyển group `arm` sang **PWM mode**), theo dõi `reference_`, nhận setpoint runtime qua topic `/rx150/fuzzy/setpoint` (`Float64MultiArray`, 5 khớp). Dùng driver classic `xs_sdk` (topic `commands/joint_group`, service `get_robot_info`/`set_operating_modes`/`torque_enable`, topic `joint_states`) — **không** ros2_control.

Vì fuzzy đặt motor ở PWM mode, không thể dùng chung `JointTrajectoryController` position mode. Giải pháp đã chốt: **một node cầu (bridge)** — làm `FollowJointTrajectory` action server cho MoveIt, lấy mẫu trajectory theo thời gian rồi publish từng điểm lên `/rx150/fuzzy/setpoint` để vòng kín PWM của fuzzy bám theo. MoveIt (planning OMPL + TOTP) và `fuzzy_node` gần như giữ nguyên. Phạm vi: **chỉ 5 khớp arm** (gripper để sau).

## Thiết kế

### Kiến trúc dữ liệu (1 tiến trình bridge)
```
move_group ──goal──▶ /rx150/arm_controller/follow_joint_trajectory  (FollowJointTrajectory action)
                         │ fuzzy_trajectory_bridge (namespace rx150)
                         │  - lấy mẫu JointTrajectory theo elapsed time (nội suy tuyến tính)
                         ▼
                   /rx150/fuzzy/setpoint  (Float64MultiArray, thứ tự [waist,shoulder,elbow,wrist_angle,wrist_rotate])
                         │ fuzzy_node (PWM closed-loop 100 Hz, watchdog zero-PWM, u_max clamp)
                         ▼
                   /rx150/commands/joint_group  (JointGroupCommand PWM) ──▶ xs_sdk ──▶ động cơ
```
Bridge sub `/rx150/joint_states` để kiểm tra goal tolerance cuối. Tên action `/rx150/arm_controller/follow_joint_trajectory` **khớp sẵn** với `interbotix_xsarm_moveit/config/controllers/rx150_controllers.yaml` (controller `/rx150/arm_controller`, action_ns `follow_joint_trajectory`) → MoveIt thấy ngay, không cần sửa config MoveIt.

## Files mới / sửa

### 1. MỚI `src/fuzzy_controller/scripts/fuzzy_trajectory_bridge` (Python/rclpy, executable)
- Node namespace `rx150`. Dùng **MultiThreadedExecutor** + reentrant callback group để vòng lặp execute không block sub `joint_states`.
- Action server tên `arm_controller/follow_joint_trajectory` (→ `/rx150/arm_controller/follow_joint_trajectory`), type `control_msgs/action/FollowJointTrajectory`.
- Publisher `fuzzy/setpoint` (`std_msgs/Float64MultiArray`).
- Subscription `/rx150/joint_states` (`sensor_msgs/JointState`), lưu state gần nhất + stamp.
- Hằng số `ARM_JOINTS = ['waist','shoulder','elbow','wrist_angle','wrist_rotate']`.
- Params: `setpoint_rate` (100.0), `default_goal_tolerance` (0.02 rad), `goal_time_margin` (0.5 s).
- `execute_callback(goal_handle)`:
  1. Lấy `trajectory` + `joint_names` từ goal. Validate `joint_names ⊆ ARM_JOINTS`; build map tên→index và map sang thứ tự arm chuẩn. Nếu có khớp ngoài arm → `REJECTED`/`ABORTED` + error string.
  2. Quy đổi `time_from_start` mỗi point sang giây (`sec + nsec*1e-9`).
  3. `rate = self.create_rate(setpoint_rate)`; `start = self.get_clock().now()`.
  4. Loop:
     - nếu `goal_handle.is_cancel_requested` → `canceled()`, return `CANCELED`.
     - `elapsed = (now - start)`. Nếu `elapsed ≤ 0` → điểm đầu; nếu `≥ duration` → điểm cuối; còn lại tìm segment `[i,i+1]`, nội suy tuyến tính `q = qi + α(qi+1 - qi)`.
     - Đóng gói 5 giá trị theo thứ tự arm, publish `Float64MultiArray`.
     - `elapsed ≥ duration + goal_time_margin` → break.
     - `rate.sleep()`.
  5. Cuối: đọc `joint_states` thực, sai số mỗi khớp vs điểm cuối. Dùng `goal_tolerance` trong goal (nếu có), ngược lại `default_goal_tolerance`. Nếu tất cả trong ngưỡng → `SUCCESS`, còn lại `GOAL_TOLERANCE_VIOLATED`. Trả `result` với `error_code`/`error_string`.
- Goal policy: chỉ 1 goal chạy; goal mới preempts (ghi nhận cancel goal cũ). Xử lý an toàn khi `joint_states` chưa đến (return error ở bước cuối, không block).
- Lý do stream cả khi fuzzy đã có closed-loop: MoveIt trajectory (TOTP) chính là **reference profile**; fuzzy chỉ cần bám điểm tham chiếu đang trôi. Lưu ý: trong `fuzzy_gains.yaml` Ruckig `enable_profile` hiện là **dead code** (chưa wire vào `onTimer`), nên không xung đột với trajectory streaming — để nguyên.

### 2. MỚI `src/fuzzy_controller/launch/fuzzy_moveit.launch.py`
Model trên `interbotix_xsarm_moveit/launch/xsarm_moveit.launch.py`, thay backend:
- **Include xsarm_control** (giống `fuzzy_control.launch.py`): `robot_model:=rx150 robot_name:=rx150 motor_configs:=<share fuzzy_controller>/config/rx150_fuzzy.yaml use_sim:=false use_rviz:=false`. Cho driver `xs_sdk` + `joint_states` + robot_description cho TF.
- **Node fuzzy_node**: namespace `rx150`, params `fuzzy_gains.yaml` (như `fuzzy_control.launch.py`).
- **Node fuzzy_trajectory_bridge**: package `fuzzy_controller`, executable `fuzzy_trajectory_bridge`, namespace `rx150`, `output='screen'`.
- **Node move_group**: tham số y hệt `xsarm_moveit.launch.py` (robot_description + semantic qua helper `interbotix_xs_modules`, kinematics/ompl/controllers/joint_limits yaml từ share `interbotix_xsarm_moveit`, planning_scene_monitor `joint_state_topic='/rx150/joint_states'`, remaps `/arm_controller/follow_joint_trajectory` → `/rx150/...`), nhưng đặt **`trajectory_execution.moveit_manage_controllers: False`** (không có controller_manager ros2_control).
- **Node rviz** (nếu `use_moveit_rviz:=true`): config `interbotix_xsarm_moveit/rviz/xsarm_moveit.rviz`.
- Reuse helper import: `from interbotix_xs_modules.xs_common import get_interbotix_xsarm_models`; `from interbotix_xs_modules.xs_launch import construct_interbotix_xsarm_semantic_robot_description_command, declare_interbotix_xsarm_robot_description_launch_arguments, determine_use_sim_time_param`.
- Args: `robot_model` (default `rx150`), `robot_name` (default `rx150`), `use_moveit_rviz` (default `true`).

### 3. SỬA `src/fuzzy_controller/CMakeLists.txt`
- Dòng `install(PROGRAMS scripts/fuzzy_gui ...)` → thêm bridge: `install(PROGRAMS scripts/fuzzy_gui scripts/fuzzy_trajectory_bridge DESTINATION lib/${PROJECT_NAME})`.
- `install(DIRECTORY config launch ...)` đã bao trùm launch mới, không cần thêm.

### 4. SỬA `src/fuzzy_controller/package.xml`
- Thêm exec/runtime deps: `<exec_depend>control_msgs</exec_depend>`, `<exec_depend>trajectory_msgs</exec_depend>`, `<exec_depend>action_msgs</exec_depend>`, `<exec_depend>interbotix_xs_modules</exec_depend>`, `<exec_depend>interbotix_xsarm_moveit</exec_depend>`, `<exec_depend>moveit_ros_move_group</exec_depend>`, `<exec_depend>interbotix_xsarm_control</exec_depend>`.

## Không động tới
- `fuzzy_node.cpp/hpp`, `fuzzy_gains.yaml`, `rx150_fuzzy.yaml`, mọi config trong `interbotix_xsarm_moveit` — dùng nguyên giá trị. (Bridge khớp tên action/controller sẵn trong `rx150_controllers.yaml`.)

## Giới hạn v1 (ghi nhận)
- Gripper: MoveIt vẫn list `gripper_controller` nhưng không có action server → lệnh gripper từ RViz fail; arm planning đầy đủ.
- Bridge chỉ phục vụ 5 khớp arm; từ chối trajectory chứa khớp ngoài arm.

## Build & kiểm chứng
1. Build: `cd ~/interbotix_ws && colcon build --packages-select fuzzy_controller && source install/setup.bash`.
2. Kiểm tra cài đặt: `ros2 pkg prefix fuzzy_controller` và `ls install/fuzzy_controller/share/fuzzy_controller/launch/` (có `fuzzy_moveit.launch.py`); `ls install/fuzzy_controller/lib/fuzzy_controller/` (có `fuzzy_trajectory_bridge`).
3. Chạy trên robot (đã cắm, đã source): `ros2 launch fuzzy_controller fuzzy_moveit.launch.py`.
4. Xác minh topic/action tồn tại: `ros2 action list` (thấy `/rx150/arm_controller/follow_joint_trajectory`), `ros2 topic info /rx150/fuzzy/setpoint`.
5. RViz MotionPlanning: đặt goal (kéo end-effector hoặc pose đã đặt tên) → **Plan & Execute**. Tay chạy qua fuzzy PWM.
6. Quan sát bám: `ros2 topic echo /rx150/fuzzy/setpoint` (reference streaming), `ros2 topic echo /rx150/fuzzy/error`, `/rx150/fuzzy/effort`.
7. Test CLI nhanh (không cần RViz): gửi 1 goal tới action `/rx150/arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) với trajectory 2 điểm → tay di chuyển.
8. An toàn: fuzzy đã có watchdog (zero PWM khi `joint_states` stale) + clamp `u_max`. Bridge chỉ stream trong profile TOTP của MoveIt (vel/accel đã giới hạn).
