# Kế hoạch: Lớp điều khiển Fuzzy cho cánh tay Interbotix RX150

## Context (bối cảnh & mục tiêu)

Workspace `/home/hust/interbotix_ws` là **ROS 2 Humble + colcon**, chứa các repo upstream của
Interbotix (bản gốc, BSD). Cánh tay rx150 dùng **động cơ Dynamixel XM540/XM430 (Protocol 2.0)**,
giao tiếp qua adapter **U2D2** (`/dev/ttyDXL`).

**Mục tiêu:** thay thế luật điều khiển vị trí mặc định (nằm sẵn trong **firmware** Dynamixel) bằng một
**bộ điều khiển fuzzy Mamdani** chạy phía máy chủ (host-side), lấy từ pipeline `.fis → C` đã có sẵn
của bạn (`gen_fit_and_3d_graph/fis2c.py` → `fuzzy_type1.c/.h`, hàm `float fuzzy_type1_eval(float e, float ed)`).

**Yêu cầu phi chức năng:** cấu trúc rõ ràng, dễ debug, dễ trình bày, không chồng lấn/lồng nhau, và
**không sửa mã upstream** (để `git pull` sạch).

**Quyết định đã chốt:** mode xuất = **PWM**; node = **C++ (ament_cmake, link trực tiếp `fuzzy_type1.c`)**;
setpoint = **pose cố định trong config** (phiên bản đầu).

---

## 1. Phân tích cấu trúc project theo cụm (đáp ứng yêu cầu trình bày)

Toàn bộ `src/` là **upstream/vendor — KHÔNG ĐƯỢC SỬA** (3 repo Interbotix bản `humble` + 1 helper MoveIt).
Sự thay đổi duy nhất hiện có chỉ là xoá file `COLCON_IGNORE` để bật build các package tùy chọn (đã có sẵn).

| Cụm / Repo | Vai trò | Được sửa? |
|---|---|---|
| `interbotix_ros_core/.../interbotix_xs_driver` | Driver C++ thấp cấp: I/O motor, đổi mode, sync read/write | ❌ vendor |
| `.../interbotix_xs_sdk` | Node ROS2 `xs_sdk` (relay lệnh + phát `joint_states` 100 Hz) | ❌ vendor |
| `.../interbotix_xs_msgs` | Định nghĩa msg/srv (`JointGroupCommand`, `OperatingModes`, `TorqueEnable`, `RobotInfo`...) | ❌ vendor — **chỉ "dùng lại" để link** |
| `.../dynamixel_workbench_toolbox` | Thư viện ROBOTIS bao DynamixelSDK | ❌ vendor |
| `interbotix_ros_manipulators/.../interbotix_xsarm_control` | Launch + config `rx150.yaml`, `modes.yaml` | ❌ vendor (chỉ **đọc/ghi runtime**, không sửa file) |
| `.../interbotix_xsarm_descriptions` | URDF/motor configs cho rx150 | ❌ vendor |
| `.../interbotix_xsarm_moveit` | Config MoveIt2 rx150 | ❌ vendor (không dùng trong v1) |
| `.../examples/interbotix_xsarm_pid` | **Pattern tham chiếu** (đang có `COLCON_IGNORE`, là code ROS1 — không build, chỉ đọc để học cấu trúc host-side controller) | ⚠️ chỉ tham khảo |
| `interbotix_ros_toolboxes/.../interbotix_xs_modules` | Python API cấp cao (`InterbotixManipulatorXS`, `core.py`) | ❌ vendor (dùng cho script test, không dùng trong node C++) |
| `test_rx150.py`, `gen_fit_and_3d_graph/`, `.vscode/` | **File của bạn** | ✅ được sửa tự do |
| **`src/fuzzy_controller/` (MỚI)** | Gói fuzzy bạn sẽ tạo | ✅ **vùng làm việc duy nhất** |

**Điểm chèn fuzzy (quan trọng):** hiện tại host **không có luật điều khiển nào** — lệnh đi thẳng thành
`Goal_Position` và vòng điều khiển kín nằm **hoàn toàn trong firmware** động cơ. Bypass firmware position
loop bằng cách đặt arm vào **`operating_mode: pwm`** (về PWM raw), khi đó firmware chỉ khuếch đại PWM theo
lệnh của bạn → fuzzy của bạn trở thành luật điều khiển thực sự. Đây là "Strategy A — host-side node, ít xâm
nhất", đúng cách `interbotix_xsarm_pid` đã làm.

---

## 2. Giao thức truyền thông & rủi ro khi đổi driver (đáp ứng yêu cầu tìm hiểu)

- **Tốc độ tối đa PC↔motor:** bảng baud Protocol 2.0 lên tới **10.5 Mbps** (reg value 8); nhưng
  **U2D2 (FTDI FT232H) thực tế giới hạn ~4.5 Mbps** (value 7). Cấu hình hiện tại = **1 Mbps** (value 3).
- **Baud hiện tại được hardcode** ở 2 nơi: `interbotix_xs_driver/.../xs_common.hpp` (`DEFAULT_BAUDRATE`
  1000000) và EEPROM motor (reg value 3). **Lưu ý bẫy:** giá trị `Baud_Rate` trong `rx150.yaml` **KHÔNG
  được ghi xuống motor** (driver bỏ qua khi `reg == "Baud_Rate"`), nó chỉ là ghi chú. Nếu muốn tăng baud
  phải (a) tự ghi EEPROM motor + (b) sửa `DEFAULT_BAUDRATE` rồi **build lại** driver → **mismatch = mất
  bus** (phải reset bằng Dynamixel Wizard). → Quy trình nâng lên **4 Mbps** (theo quyết định của bạn) nằm
  ở **mục 7**, tách biệt khỏi gói fuzzy. **Lưu ý:** 4 Mbps nằm sát giới hạn thực tế của U2D2 — nếu không
  giữ được 100 Hz thì lùi về 3 Mbps (value 5, điểm ngọt an toàn).
- **Tốc độ vòng lặp:** `joint_states` (SyncRead) mặc định **100 Hz** (`update_rate` trong `rx150.yaml`);
  lệnh ghi là **event-driven** (pub nhanh đến đâu ghi nhanh đến đó). Với **syncRead thường**, trần thực tế trên
  U2D2 ≈ **200–250 Hz** (sàn FTDI latency 1 ms + round-trip USB). Với **FastSyncRead (mục 8)** trần đẩy lên
  ~**500–800 Hz**. Toán fuzzy **không** phải giới hạn (rẻ như ~kHz). **Dự án target 500 Hz** (cả `update_rate`
  telemetry lẫn `loop_rate` fuzzy) — cần FastSyncRead + baud cao (mục 7); nếu lỗi packet thì lùi 300 Hz.
- **Đổi driver setting — rủi ro:**
  - Tham số **RAM** trong `modes.yaml` (`operating_mode`, `profile_type`, `profile_velocity`,
    `profile_acceleration`, `torque_enable`): đổi runtime qua service, **không cần restart**, nhưng đổi
    `operating_mode`/`profile_type` sẽ **tắt torque chốc lát** → phải để arm ở **sleep pose** (chống trọng
    lực) trước khi đổi. Đây là chính xác những gì node fuzzy sẽ gọi.
  - Tham số **EEPROM** trong `rx150.yaml` (ID, Baud_Rate, Drive_Mode, Velocity_Limit, Min/Max_Position...) :
    cần torque-off + restart để áp dụng lại; **đừng đụng**.
  - Driver **không kiểm soát giới hạn vị trí** (URDF/MoveIt mới là rào an toàn) → fuzzy phải tự giới hạn.
  - `Resolution_Divider` **không có** trên XM540/XM430 (chỉ MX-series) → không liên quan.

---

## 3. Kiến trúc giải pháp (C++/PWM/fixed-pose)

**Một package mới, biệt lập, không lồng vào vendor:**

```
src/fuzzy_controller/                         # ament_cmake
├── package.xml                               # depend: rclcpp, sensor_msgs, interbotix_xs_msgs
├── CMakeLists.txt                            # build lib fuzzy_type1 + node fuzzy_node
├── README.md                                 # mô tả + cách chạy + cách regenerate
├── include/fuzzy_controller/
│   └── fuzzy_node.hpp
├── src/
│   ├── fuzzy_node.cpp                        # node rclcpp (timer 100Hz + toàn bộ logic)
│   └── fuzzy_type1.c                         # copy từ gen_fit_and_3d_graph (AUTO-GENERATED)
├── generated/
│   ├── fuzzy_type1.fis                       # nguồn sự thật của luật fuzzy
│   ├── fuzzy_type1.h                         # copy (extern "C", float fuzzy_type1_eval(e, ed))
│   └── regenerate.sh                         # gọi: python3 ../../gen_fit_and_3d_graph/fis2c.py ...
├── config/
│   ├── fuzzy_gains.yaml                      # Ke,Ked,u_max/joint, loop_rate, reference_pose, safety
│   └── rx150_fuzzy.yaml                      # copy rx150.yaml upstream, chỉ đổi update_rate: 500
└── launch/
    └── fuzzy_control.launch.py               # include xsarm_control + chạy fuzzy_node
```

**Tách bạch rõ 3 tầng (không chồng lấn):**
1. **Tầng luật fuzzy (thuần toán):** chỉ là `fuzzy_type1.c/.h` — nguồn `.fis`, sửa bằng `fis2c.py`, không
   lẫn ROS. Dễ trình bày/đối chiếu mặt phẳng điều khiển (`fuzzy_surface.html` đã có).
2. **Tầng giao tiếp ROS (node):** `fuzzy_node.cpp` — chỉ lo subscribe `joint_states`, gọi service đổi mode,
   pub `commands/joint_group` + topic debug. Không chứa toán fuzzy.
3. **Tầng tham số (config):** `fuzzy_gains.yaml` — scaling vật lý (Ke/Ked/u_max theo khớp), reference,
   safety. Đổi hành vi mà không build lại.

### Vòng điều khiển trong `fuzzy_node.cpp` (timer 500 Hz — FastSyncRead)
1. **Khởi tạo:** đọc param (`robot_name`, `group_name`, `loop_rate`, `reference_pose`, `Ke[]`, `Ked[]`,
   `u_max[]`, `watchdog_timeout_s`). Gọi service **`/<robot>/get_robot_info`** (`cmd_type="group"`,
   `name="arm"`) để lấy **thứ tự khớp chuẩn** và **sleep positions** → tránh hardcode.
2. **Chuyển mode:** gọi **`/<robot>/set_operating_modes`** (`cmd_type="group"`, `name="arm"`, `mode="pwm"`,
   `profile_velocity=0`, `profile_acceleration=0`). Đảm bảo torque on qua **`/<robot>/torque_enable`**.
3. **Subscribe** `/<robot>/joint_states` (`sensor_msgs/JointState`); map tên→chỉ số.
4. **Mỗi tick (500 Hz), cho mỗi khớp arm** (theo thứ tự từ `get_robot_info`):
   - `e   = reference_pose[i] - pos[i]`        (rad)
   - `ed  = -vel[i]`                            (rad/s; dùng telemetry, ổn định hơn đạo hàm rời rạc)
   - chuẩn hoá (giới hạn [-1,1]): `en  = clamp(Ke[i]*e,   -1, 1)`; `edn = clamp(Ked[i]*ed, -1, 1)`
   - `un  = fuzzy_type1_eval(en, edn)`          (∈ [-1,1], gọi hàm C đã sinh)
   - `u   = un * u_max[i]`                       (PWM raw, có dấu)
   - gom vào `cmd[]`.
5. **Pub** `/<robot>/commands/joint_group` (`JointGroupCommand{name="arm", cmd=[...]}`).
6. **Debug (rất quan trọng để trình bày/tuning):** pub thêm 3 `sensor_msgs/JointState`:
   `/<robot>/fuzzy/error`, `/<robot>/fuzzy/edot`, `/<robot>/fuzzy/effort` → vẽ trực tiếp bằng
   `rqt_plot`/PlotJuggler (đáp ứng "dễ debug, dễ trình bày").
7. **An toàn:**
   - **Watchdog:** nếu `joint_states` cũ quá `watchdog_timeout_s` → pub PWM = 0 + log ERROR.
   - **Shutdown (SIGINT/h Destructor):** pub PWM = 0 rồi gọi `torque_enable(..., false)` → không giữ PWM cũ.

### `config/fuzzy_gains.yaml` (ví dụ)
```yaml
fuzzy_controller:
  ros__parameters:
    robot_name: rx150
    group_name: arm
    loop_rate: 500.0   # = update_rate; cần FastSyncRead (mục 8). Nếu lỗi packet → lùi cả hai về 300
    # Pose tham chiếu (rad), theo thứ tự khớp arm: [waist, shoulder, elbow, wrist_angle, wrist_rotate].
    # Bỏ trống [] → tự lấy sleep_positions từ get_robot_info (mặc định an toàn).
    reference_pose: [0.0, -1.80, 1.55, 0.8, 0.0]
    gains:
      Ke:   [2.0, 2.0, 2.0, 2.5, 2.5]    # 1/rad — chuẩn hoá lỗi góc
      Ked:  [0.05, 0.05, 0.05, 0.05, 0.05]  # s/rad — chuẩn hoá đạo hàm lỗi
      u_max: [600, 800, 800, 600, 600]    # PWM raw cap (giới hạn XM = ±885; để dư biên an toàn)
    safety:
      watchdog_timeout_s: 0.2
      zero_on_shutdown: true
```

### `CMakeLists.txt` (yếu tố then chốt)
```cmake
add_library(fuzzy_type1 STATIC src/fuzzy_type1.c)
target_link_libraries(fuzzy_type1 m)        # -lm (centroid defuzz dùng pow/sqrt)
target_include_directories(fuzzy_type1 PUBLIC generated)

add_executable(fuzzy_node src/fuzzy_node.cpp)
ament_target_dependencies(fuzzy_node rclcpp sensor_msgs interbotix_xs_msgs)
target_link_libraries(fuzzy_node fuzzy_type1)
target_include_directories(fuzzy_node PRIVATE include/fuzzy_controller)
```
> Kiểm tra lại: xác minh `fuzzy_type1.c` chỉ phụ thuộc `<math.h>` (không include lạ) và export đúng
> `fuzzy_type1_eval(float,float)` — đã xác nhận qua `fuzzy_type1.h`.

### `launch/fuzzy_control.launch.py`
- Include `xsarm_control.launch.py` (`robot_model:=rx150`), **đè `motor_configs`** trỏ tới `config/rx150_fuzzy.yaml`
  (bản copy upstream rx150.yaml, **chỉ đổi `update_rate: 500``) → nâng tần telemetry. **Yêu cầu kèm FastSyncRead
  (mục 8)** thì 500 Hz mới đạt được; không sửa rx150.yaml/modes.yaml upstream.
- Sinh node `fuzzy_controller` với `parameters=[fuzzy_gains.yaml]` (`loop_rate=500`), `node_namespace=/<robot_name>`,
  `output=screen`. **`loop_rate` phải bằng `update_rate`** để không thừa chu kỳ dùng số liệu cũ.
- (Tuỳ chọn) tham số `start_sleep_pose:=true` để trước khi bật fuzzy, đưa arm về sleep (an toàn khi đổi mode).

### Tái dùng (KHÔNG viết lại)
- Msg/srv: `interbotix_xs_msgs` (`JointGroupCommand`, `OperatingModes`, `TorqueEnable`, `RobotInfo`) — link.
- `sensor_msgs/JointState` cho cả telemetry lẫn debug.
- Node `xs_sdk` chạy **nguyên vẹn**, chỉ đổi mode runtime → **không sửa file upstream nào**.
- `fis2c.py` (đã có) để tái sinh `fuzzy_type1.c` khi sửa `.fis`.

---

## 4. Các file sẽ tạo / chạm
- **Tạo mới (toàn bộ trong `src/fuzzy_controller/`):** danh sách cây file ở mục 3.
- **Sửa upstream:** **Tối thiểu — 1 patch có tài liệu.** Mọi thay đổi mode/tham số runtime qua service; configs
  (`rx150.yaml`/`modes.yaml`) giữ nguyên (dùng bản copy `config/rx150_fuzzy.yaml`). **Ngoại lệ duy nhất:** bật
  FastSyncRead cần sửa 1 dòng trong `dynamixel_workbench_toolbox` (submodule vendor) — xem mục 8, giữ như patch
  cục bộ (áp lại khi `git pull`).
- **File của bạn có thể cần chỉnh nhẹ:** `gen_fit_and_3d_graph/fis2c.py` (nếu cần đường dẫn output) — chỉ copy
  sản phẩm vào `generated/` qua `regenerate.sh`.

---

## 5. Verification (kiểm thử end-to-end)

1. **Tái sinh luật:** `cd src/fuzzy_controller/generated && ./regenerate.sh` → đối chiếu `fuzzy_type1.c`
   với `gen_fit_and_3d_graph/fuzzy_type1.c` (phải giống).
2. **Build:** `source install/setup.bash && colcon build --packages-select fuzzy_controller` (build sạch,
   không lỗi link `-lm`).
3. **Chạy arm thật:** launch qua `fuzzy_control.launch.py` (đã đè `update_rate=500` + FastSyncRead mục 8) →
   kiểm `ros2 topic hz /rx150/joint_states` ≈ **500 Hz** (nếu tụt/nhiều lỗi packet: lùi `update_rate`/`loop_rate`
   về 300, kiểm tra patch FastSyncRead + baud mục 7); đưa arm về sleep pose.
4. **Kích hoạt fuzzy:** `ros2 launch fuzzy_controller fuzzy_control.launch.py` rồi xác nhận:
   - `ros2 service call /rx150/get_robot_info interbotix_xs_msgs/RobotInfo '{cmd_type: "group", name: "arm"}'`
     → thấy mode = `pwm`.
   - `ros2 topic echo /rx150/commands/joint_group` → thấy giá trị PWM raw (có dấu).
   - Arm **giữ pose tham chiếu**; đẩy nhẹ một khớp → thấy nó kéo về (chứng tỏ vòng kín fuzzy hoạt động).
5. **Debug/tuning:** `rqt_plot /rx150/fuzzy/error/position[0] /rx150/fuzzy/effort/effort[0]` → xem đáp ứng,
   chỉnh `Ke/Ked/u_max` trong `fuzzy_gains.yaml`, relaunch.
6. **Step response (để trình bày):** đổi `reference_pose` một khớp (vd wrist +0.5 rad) → quan sát quá độ/
   settling qua topic debug → chụp cho báo cáo.
7. **An toàn:** `Ctrl+C` node → xác nhận pub PWM = 0 và torque off (arm rơi tự do an toàn ở sleep pose).
8. **Watchdog:** kill node `xs_sdk` đang fuzzy chạy → fuzzy node phải phát hiện stale `joint_states`, pub 0,
   log ERROR.

---

## 6. Rủi ro & lưu ý an toàn
- **Đổi mode pwm tắt torque chốc lát** → bao giờ cũng khởi động ở **sleep pose** (chống trọng lực).
- **PWM mode = không có giới hạn vị trí nội tại** → `u_max` phải đủ nhỏ khi mới tune; tăng dần.
- **Trọng lực:** PWM thuần là duty-cycle, không phải torque → các khớp nâng tải (shoulder/elbow) cần `u_max`
  và `Ke` riêng lớn hơn. Đây là lý do config tách `Ke/Ked/u_max` theo khớp.
- **Tốc độ vòng:** dự án target **500 Hz** (cả `update_rate` + `loop_rate`) nhờ FastSyncRead (mục 8). Phải đi
  kèm baud cao (3–4 Mbps, mục 7) và canh lỗi packet; sàn FTDI 1 ms vẫn giới hạn — nếu 500 Hz không ổn thì lùi 300 Hz.
- **Đổi baud là rủi ro brick bus** → chỉ làm theo **mục 7**, có sẵn đường lùi về 3 Mbps và khôi phục.

---

## 7. (Tùy chọn) Nâng baud bus lên 4 Mbps — độc lập với gói fuzzy

Quy trình "cutover" đổi **cả 2 phía** (làm một lần, tách rời việc build fuzzy). **Tất cả motor trên cùng
bus phải chung 1 baud** — rx150 có 6 motor ID 1–6 daisy-chain trên `/dev/ttyDXL`, phải đổi đồng loạt.

### Phase 0 — chuẩn bị (ở 1 Mbps hiện tại)
- `xs_sdk` chạy ngon; `ros2 topic hz /rx150/joint_states` ≈ 100 Hz. Ghi lại 6 ID, baud hiện tại = value `3`.

### Phase 1 — đổi EEPROM motor → value `6` (4 Mbps)
**Cách an toàn nhất — Dynamixel Wizard 2.0:** chọn port `/dev/ttyDXL` @ 1 Mbps, scan thấy 6 motor. Với từng
motor: torque off → set thanh ghi `Baud_Rate` (addr 8) = `6` → motor "biến mất" khỏi view 1 Mbps (bình
thường). Làm đủ 6 motor. Đặt port Wizard = 4 000 000 bps, scan lại → phải thấy cả 6.

**Hoặc qua service** (đổi hàng loạt rồi cắt host): `ros2 service call /rx150/torque_enable ...` (off hết) →
`ros2 service call /rx150/set_motor_registers interbotix_xs_msgs/RegisterValues "{cmd_type: 'group',
name: 'arm', reg: 'Baud_Rate', value: 6}"` (làm tương tự cho gripper) → toàn bộ motor rớt khỏi bus 1 Mbps
ngay lập tức (đúng như mong đợi).

> **Bẫy lặp lại:** giá trị `Baud_Rate` trong `rx150.yaml` **không** ghi xuống motor. Sau khi đổi, cập nhật
> `Baud_Rate: 6` trong `rx150.yaml` **chỉ để tài liệu khớp**, không phải để áp dụng.

### Phase 2 — đổi baud host + rebuild
- Sửa `src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_driver/include/interbotix_xs_driver/xs_common.hpp`
  (~dòng 44): `#define DEFAULT_BAUDRATE 4000000`.
- `colcon build --packages-up-to interbotix_xs_sdk` (rebuild driver + sdk). `source install/setup.bash`.

### Phase 3 — xác minh ở 4 Mbps
- Relaunch `xs_sdk`. `ros2 topic hz /rx150/joint_states` phải vẫn ≈ **100 Hz**, không cảnh báo read-failure
  liên tục.
- **Tiêu chí thất bại:** nếu tần số tụt hoặc log hiện ping-scan lỗi → **U2D2 không giữ nổi 4 Mbps** → lùi
  về **value 5 (3 Mbps)**: lặp Phase 1 đặt value `5` + sửa `DEFAULT_BAUDRATE 3000000` + rebuild.

### Phase 4 — khôi phục nếu mất bus
- Dynamixel Wizard: đặt port baud = 4 000 000, scan; thấy motor → set `Baud_Rate` về `3` để hoàn nguyên.
- Nếu Wizard cũng không thấy → motor đang ở baud khác → quét lần lượt 1M / 2M / 3M / 4M / 4.5M đến khi bắt
  được motor, rồi đặt lại về giá trị mong muốn.

---

## 8. Kích hoạt FastSyncRead + nâng tần số vòng lên ~500 Hz (mặc định từ đầu)

`GroupFastSyncRead` đã có sẵn trong SDK hệ thống (`/opt/ros/humble/include/dynamixel_sdk/group_fast_sync_read.h`),
là subclass của `GroupSyncRead` **cùng constructor & API** → đổi là **drop-in**. (Chỉ hỗ trợ Protocol 2.0 — rx150 thoả.)
Driver Interbotix hiện đang dùng `GroupSyncRead` thường (`dynamixel_driver.cpp` `addSyncReadHandler`).

### Sửa 1 dòng trong vendor (patch cục bộ, có tài liệu)
File: `src/interbotix_ros_core/interbotix_ros_xseries/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_driver.cpp`
- Thêm include: `#include "dynamixel_sdk/group_fast_sync_read.h"`
- Trong `addSyncReadHandler` (2 chỗ, ~dòng 1057 & 1086) đổi:
  `new dynamixel::GroupSyncRead(portHandler_, packetHandler_, address, length)`
  → `new dynamixel::GroupFastSyncRead(portHandler_, packetHandler_, address, length)`.
- Phần còn lại của Interbotix (`getSyncReadData`, `isAvailable`…) gọi trên class cơ sở → **chạy y nguyên**.

### Rebuild + chạy
- `colcon build --packages-up-to interbotix_xs_sdk` (rebuild workbench toolbox + driver + sdk). `source install/setup.bash`.
- Đặt `update_rate: 500` (`config/rx150_fuzzy.yaml`) và `loop_rate: 500` (`fuzzy_gains.yaml`).
- **Đi kèm baud 4 Mbps (mục 7)** — FastSyncRead chỉ tối ưu phần nhận; băng thông truyền vẫn cần baud cao để 500 Hz không nghẽn.

### Xác minh & lùi
- `ros2 topic hz /rx150/joint_states` ≈ **500 Hz**; log `xs_sdk` không có read-failure/ping-scan liên tục.
- Nếu không đạt hoặc lỗi packet nhiều → lùi `update_rate`/`loop_rate` về **300 Hz**; vẫn không ổn → kiểm tra lại patch và/hoặc giảm baud.
- **Giữ patch như commit/fork cục bộ**: ghi note để áp lại khi `git pull` submodule workbench toolbox.

### Lưu ý
- Đây là **điểm duy nhất sửa vendor** trong toàn dự án (xem mục 4). Sàn FTDI latency 1 ms vẫn còn nên ~500–800 Hz là vùng khả thi, không phải "1 kHz sạch".
- Thứ tự thực thi khuyến nghị: **(1)** baud 4 Mbps (mục 7) → **(2)** patch FastSyncRead (mục này) → **(3)** build & chạy gói fuzzy (mục 3–6).
