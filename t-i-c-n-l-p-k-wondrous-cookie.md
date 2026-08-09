# Bộ điều khiển Fuzzy host-side cho Interbotix RX150 — Spec triển khai MVP

> Phạm vi MVP: chứng minh bộ điều khiển fuzzy Mamdani hoạt động ổn định ở 100 Hz, **không sửa mã
> upstream và không thay baud bus**. Việc nâng tần số vòng lặp và fail-safe phần cứng được tách sang
> Mục 10 (sau MVP).

## 1. Bối cảnh và phạm vi

Workspace `/home/hust/interbotix_ws` dùng ROS 2 Humble + colcon và chứa bốn Git repository độc lập
trong `src/`; workspace root không phải một Git repository. Các repo Interbotix hiện đã có thay đổi
cục bộ (một số file `COLCON_IGNORE` đã bị xoá), nên mọi thay đổi mới phải tránh chạm vào phần vendor.

Mục tiêu: thay vòng điều khiển vị trí trong firmware Dynamixel bằng bộ điều khiển fuzzy Mamdani chạy
trên host. Pipeline luật fuzzy đã có sẵn:

```text
fuzzy_type1.fis -> fis2c.py -> fuzzy_type1.c/.h
                               fuzzy_type1_eval(e, ed)
```

Quyết định cho MVP:

- Output điều khiển là **Goal PWM raw có dấu**.
- Controller viết bằng C++/`rclcpp`; luật fuzzy sinh ra vẫn là C99.
- Setpoint là một pose cố định trong YAML.
- Telemetry và vòng điều khiển chạy theo `joint_states` mặc định khoảng 100 Hz.
- Không sửa driver, SDK, workbench toolbox, config upstream, baud EEPROM hay firmware.
- Controller mặc định khởi động ở trạng thái `DISARMED`; người vận hành phải kích hoạt rõ ràng.

> **Bắt buộc trước khi cấp PWM:** model motor, firmware và giới hạn PWM **không** được suy ra từ tên
> robot. Phải đọc trực tiếp các register trên từng ID và lưu kết quả vào biên bản thử nghiệm.

## 2. Vị trí tích hợp trong project

| Thành phần | Vai trò | Chính sách |
|---|---|---|
| `interbotix_xs_driver` | I/O Dynamixel, đổi đơn vị, sync read/write | chỉ đọc |
| `interbotix_xs_sdk` | nhận command ROS, phát `joint_states`, cung cấp service | chỉ dùng API |
| `interbotix_xs_msgs` | `JointGroupCommand`, `RobotInfo`, `OperatingModes`, `RegisterValues`, `TorqueEnable` | dependency |
| `interbotix_xsarm_control` | launch và config RX150 mặc định | include nguyên trạng |
| `interbotix_xsarm_descriptions` | URDF và joint limits | lấy limit qua `RobotInfo` |
| `interbotix_xsarm_pid` | ví dụ ROS 1 về host-side PWM PID | chỉ tham khảo |
| `gen_fit_and_3d_graph/` | nguồn generator `.fis→C` và công cụ vẽ mặt fuzzy | code của dự án |
| `src/fuzzy_controller/` | package mới | vùng triển khai |

Cách chèn fuzzy: ở position mode, `JointGroupCommand` được đổi thành Goal Position và firmware đóng
vòng vị trí. Ở PWM mode, driver chuyển trực tiếp giá trị float sang số nguyên PWM tại
`InterbotixDriverXS::write_commands`, nên controller host-side thật sự quyết định luật phản hồi.

> **Xung đột command:** không được chạy MoveIt, `test_rx150.py`, demo Python hay bất kỳ publisher nào
> khác tới `/<robot>/commands/joint_group` khi fuzzy đang `ACTIVE`. Một command vị trí từ node khác
> lúc arm đang ở PWM mode sẽ bị hiểu thành PWM raw.

## 3. Kiến trúc package mới

```text
src/fuzzy_controller/
├── package.xml
├── CMakeLists.txt
├── README.md
├── include/fuzzy_controller/
│   └── fuzzy_node.hpp
├── src/
│   └── fuzzy_node.cpp
├── fis/
│   └── fuzzy_type1.fis              # nguồn sự thật duy nhất của luật fuzzy
├── generated/
│   ├── fuzzy_type1.c                # AUTO-GENERATED, build trực tiếp
│   └── fuzzy_type1.h                # AUTO-GENERATED
├── scripts/
│   └── regenerate.sh
├── config/
│   └── fuzzy_controller.yaml
├── launch/
│   └── fuzzy_control.launch.py
└── test/
    └── test_fuzzy_type1.cpp
```

Chỉ giữ **một** bản `fuzzy_type1.c` (trong `generated/`); không sao chép sang `src/`, vì hai bản
generated C rất dễ lệch nhau.

`regenerate.sh` tự xác định package/workspace qua đường dẫn của chính script rồi gọi:

```text
python3 <workspace>/gen_fit_and_3d_graph/fis2c.py \
    <package>/fis/fuzzy_type1.fis -o <package>/generated
```

Sau đó xoá file demo do generator sinh (`fuzzy_type1_demo.c` / `fuzzy_type1_demo`) và chạy một kiểm
tra số học ngắn. Không dùng đường dẫn tương đối cố định kiểu `../../gen_fit_and_3d_graph` (sai cấp
thư mục).

Dependency dự kiến:

- Build/runtime: `rclcpp`, `sensor_msgs`, `std_msgs`, `std_srvs`, `interbotix_xs_msgs`.
- Launch runtime: `launch`, `launch_ros`, `interbotix_xsarm_control`.
- Test: `ament_cmake_gtest`.

`CMakeLists.txt` phải build và install đầy đủ:

```cmake
add_library(fuzzy_type1 STATIC generated/fuzzy_type1.c)
target_include_directories(fuzzy_type1 PUBLIC generated)
target_link_libraries(fuzzy_type1 PUBLIC m)

add_executable(fuzzy_node src/fuzzy_node.cpp)
ament_target_dependencies(
  fuzzy_node
  rclcpp sensor_msgs std_msgs std_srvs interbotix_xs_msgs)
target_link_libraries(fuzzy_node fuzzy_type1)
target_include_directories(fuzzy_node PRIVATE include)

install(TARGETS fuzzy_type1 fuzzy_node
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config fis DESTINATION share/${PROJECT_NAME})
install(FILES generated/fuzzy_type1.h
  DESTINATION include/${PROJECT_NAME})
```

## 4. State machine của controller

Node **không** tự đổi sang PWM ngay khi khởi động. State machine tối thiểu:

```text
STARTUP
  -> WAITING_FOR_SERVICES
  -> WAITING_FOR_JOINT_STATE
  -> PREFLIGHT
  -> DISARMED
  -> ARMING
  -> ACTIVE
  -> FAULT hoặc DISARMED
```

### 4.1 STARTUP / PREFLIGHT

1. Đọc và kiểm tra kích thước toàn bộ parameter vector.
2. Chờ có các service của `xs_sdk` với timeout hữu hạn; không block vô hạn trong constructor.
3. Gọi `get_robot_info` cho group `arm` để lấy:
   - `joint_names` và `joint_state_indices`;
   - `joint_sleep_positions`;
   - lower/upper/velocity limits từ URDF.
4. Chờ một `JointState` hợp lệ: đủ `name`, `position`, `velocity`, mọi giá trị finite.
5. Kiểm tra `reference_pose` nằm trong joint limits với safety margin.
6. Kiểm tra topic command không có publisher cạnh tranh. Sau khi node tạo publisher của chính nó, số
   publisher dự kiến là **một**.
7. Đọc register cho group `arm` và ghi log: `Model_Number`, `Firmware_Version`, `Operating_Mode`,
   `Drive_Mode`, `PWM_Limit`, `Baud_Rate`.
8. Từ chối arm nếu response thiếu giá trị, số motor không khớp, hoặc `u_max` vượt `PWM_Limit` thực tế.

> **Service không trả trạng thái:** `OperatingModes` và `TorqueEnable` không có field response, nên
> kết quả service không đủ để xác nhận phần cứng. Mọi thay đổi mode phải đọc ngược bằng
> `get_motor_registers`.

### 4.2 DISARMED

- Không phát lệnh fuzzy định kỳ.
- Arm giữ mode hiện tại (ban đầu là position mode từ config upstream).
- Cung cấp service `~/arm` kiểu `std_srvs/srv/SetBool`.
- Chỉ chấp nhận `data=true` nếu robot đang gần `reference_pose`/sleep pose trong
  `arming_pose_tolerance_rad`, state còn mới và không có publisher cạnh tranh.
- `auto_arm` mặc định `false`; không bật trong MVP.

### 4.3 ARMING

1. Người vận hành phải đỡ cơ khí arm và chuẩn bị E-stop.
2. Gọi `set_operating_modes`:

   ```text
   cmd_type=group
   name=arm
   mode=pwm
   profile_type=velocity
   profile_velocity=0
   profile_acceleration=0
   ```

3. Đọc lại `Operating_Mode`; chỉ tiếp tục nếu tất cả motor trả `16` (PWM).
4. Gửi command zero trước, sau đó ramp output từ zero trong `arm_ramp_s`; không áp bias/PWM lớn ở mẫu đầu.
5. Xác định chiều điều khiển từng khớp bằng xung PWM rất nhỏ khi tune lần đầu. `output_sign` là
   parameter hiệu chỉnh thực nghiệm; không đoán dấu chỉ từ tên joint hay `Drive_Mode`.

### 4.4 ACTIVE

Điều khiển được tính **đúng một lần** cho mỗi `JointState` mới, thay vì timer 100 Hz độc lập. Một
watchdog timer riêng chỉ kiểm tra độ mới của telemetry — cách này tránh tính nhiều lần trên cùng một mẫu.

Với mỗi joint:

```text
e       = reference_position - measured_position
ed      = -filtered_velocity
en      = clamp(Ke  * e,  -1, 1)
edn     = clamp(Ked * ed, -1, 1)
u_fuzzy = fuzzy_type1_eval(en, edn) * u_max
u_joint = gravity_bias + u_fuzzy
u_joint = apply_soft_position_limit(u_joint)
u_raw   = output_sign * slew_limit(u_joint)
u_raw   = clamp(u_raw, -pwm_cap, pwm_cap)
```

`gravity_bias` là PWM feedforward hằng theo từng joint, phù hợp với fixed-pose. Lý do bắt buộc: luật
fuzzy hiện cho output gần zero tại `e=0, ed=0`, trong khi shoulder/elbow vẫn cần mô-men chống trọng lực.
Sau MVP có thể thay bias hằng bằng gravity model theo pose hoặc thêm integral với anti-windup.

Velocity được lọc low-pass trước khi đưa vào fuzzy. Output có slew-rate limit để tránh bước PWM đột ngột.

### 4.5 FAULT / DISARM

Điều kiện fault tối thiểu:

- `joint_states` quá `state_timeout_s`;
- NaN/Inf, thiếu joint, hoặc thứ tự/index không hợp lệ;
- vượt hard joint limit;
- vượt giới hạn velocity/current đã cấu hình;
- xuất hiện thêm publisher trên command topic;
- mode readback không còn là PWM khi đang ACTIVE.

Khi fault, node cố gắng gửi zero PWM nhiều lần rồi chuyển sang `FAULT`. Đây chỉ là **best effort**:

- Nếu fuzzy node crash/SIGKILL thì nó không thể gửi zero.
- Nếu `xs_sdk` chết thì zero publish không còn được chuyển xuống bus.
- Dynamixel không có command-timeout trong kiến trúc hiện tại, nên có thể giữ Goal PWM cuối.

> **Giới hạn fail-safe quan trọng:** watchdog ROS này **không** phải một fail-safe hoàn chỉnh. MVP bắt
> buộc có người vận hành, arm được đỡ khi thử, và E-stop/cắt nguồn phần cứng trong tầm tay. Một
> fail-safe đúng cần watchdog nằm trong process trực tiếp sở hữu bus hoặc trong firmware/thiết bị ngoài
> (xem Mục 10.3).

`data=false` trên service `~/arm` gửi zero và chuyển về `DISARMED`. Không tự torque-off hay tự chuyển
sang position mode trong destructor (arm có thể rơi/giật nếu đang ở pose bất kỳ). Quy trình dừng có
kiểm soát ở Mục 8E.

## 5. Parameter ban đầu

Các số dưới đây là **giới hạn thử nghiệm bảo thủ**, không phải gain đã tune:

```yaml
fuzzy_controller:
  ros__parameters:
    robot_name: rx150
    group_name: arm
    auto_arm: false

    # [] nghĩa là dùng sleep positions từ RobotInfo.
    reference_pose: []

    gains:
      Ke:             [1.0, 1.0, 1.0, 1.0, 1.0]
      Ked:            [0.10, 0.10, 0.10, 0.10, 0.10]
      u_max:          [60.0, 100.0, 100.0, 60.0, 40.0]
      gravity_bias:   [0.0, 0.0, 0.0, 0.0, 0.0]
      output_sign:    [1.0, 1.0, 1.0, 1.0, 1.0]

    filters:
      velocity_cutoff_hz: 10.0
      pwm_slew_rate_per_s: [500.0, 500.0, 500.0, 500.0, 500.0]
      arm_ramp_s: 2.0

    safety:
      startup_timeout_s: 10.0
      state_timeout_s: 0.10
      arming_pose_tolerance_rad: 0.08
      soft_limit_margin_rad: 0.15
      hard_limit_margin_rad: 0.03
      max_abs_velocity_rad_s: [0.5, 0.5, 0.5, 0.7, 0.7]
      # Phải điền sau khi đọc model và đo baseline; [] tạm thời vô hiệu hoá check current.
      max_abs_current_ma: []
```

`u_max` còn phải bị cap động bởi `PWM_Limit` đọc từ motor. Tune từng joint với đầu arm không mang tải,
tăng dần từ giá trị nhỏ; **không** bắt đầu bằng 600–800 PWM.

## 6. Launch

`fuzzy_control.launch.py`:

- Include `interbotix_xsarm_control/launch/xsarm_control.launch.py` với `robot_model=rx150`.
- Dùng nguyên `rx150.yaml`, `modes.yaml`, baud 1 Mbps và `update_rate: 100` upstream.
- Không launch MoveIt, demo hay script điều khiển khác.
- Tạo `fuzzy_node` bằng keyword `namespace=robot_name`, không dùng `node_namespace`.
- Load YAML bằng đường dẫn từ `FindPackageShare('fuzzy_controller')`.
- Có argument `launch_driver`, `robot_name`, `use_rviz`; `auto_arm` vẫn mặc định `false`.

MVP không cần tạo `rx150_fuzzy.yaml`, nhờ đó tránh copy cả motor config EEPROM và tránh để config
upstream/config cục bộ lệch nhau.

## 7. Debug và dữ liệu trình bày

Node publish:

- `/<robot>/fuzzy/error` (`sensor_msgs/JointState`);
- `/<robot>/fuzzy/edot` (`sensor_msgs/JointState`);
- `/<robot>/fuzzy/effort` (`sensor_msgs/JointState`, chứa PWM command raw);
- `/<robot>/fuzzy/state` (`std_msgs/String`: `DISARMED`, `ACTIVE`, `FAULT`…);
- `/<robot>/fuzzy/reference` (`sensor_msgs/JointState`).

Log khi arming phải in joint order, model/register readback, PWM caps và reference. Khi fault, log
nguyên nhân **một lần** rồi throttle các log lặp lại.

Dữ liệu báo cáo:

- mặt fuzzy từ `.fis`;
- reference, position, error, velocity và PWM theo thời gian;
- tần số `joint_states` thực đo;
- overshoot, settling time, steady-state error;
- so sánh khi `gravity_bias=0` và khi đã tune bias.

## 8. Trình tự triển khai và kiểm thử

### A. Kiểm thử không có robot

1. Chạy `scripts/regenerate.sh` hai lần; xác nhận output lần hai không đổi.
2. Thêm test các điểm đặc trưng, tối thiểu:

   ```text
   f(0, 0) ~= 0
   f(+e, 0) > 0
   f(-e, 0) < 0
   f(0, +ed) > 0
   f(0, -ed) < 0
   output finite và nằm trong [-1, 1] trên grid
   ```

3. So sánh evaluator C với evaluator Python trong `gen_surface.py` trên một grid nhỏ, với tolerance
   xác định.
4. Build:

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select fuzzy_controller
   source install/setup.bash
   colcon test --packages-select fuzzy_controller
   colcon test-result --verbose
   ```

### B. Preflight robot ở position mode

1. Chạy launch Interbotix mặc định; xác nhận arm hoạt động ở position mode, 1 Mbps, khoảng 100 Hz.
2. Đưa arm về sleep/reference pose bằng API position hiện có rồi dừng publisher điều khiển đó.
3. Kiểm tra không còn publisher cạnh tranh:

   ```bash
   ros2 topic info -v /rx150/commands/joint_group
   ```

4. Đọc register, ví dụ:

   ```bash
   ros2 service call /rx150/get_motor_registers \
     interbotix_xs_msgs/srv/RegisterValues \
     "{cmd_type: group, name: arm, reg: Operating_Mode}"
   ```

5. Ghi lại `Model_Number`, `Firmware_Version`, `PWM_Limit`, `Drive_Mode`, `Baud_Rate` cho từng joint.

### C. Test PWM từng joint

1. Đỡ arm cơ khí, tháo tải, chuẩn bị E-stop.
2. Launch fuzzy controller; xác nhận state `DISARMED`.
3. Arm controller bằng service chỉ sau khi preflight pass.
4. Lần đầu, command tất cả joint bằng zero rồi tạo xung rất nhỏ cho **duy nhất một** joint để xác định
   `output_sign`. Không test cả năm joint cùng lúc.
5. Xác nhận readback `Operating_Mode=[16,16,16,16,16]` trước khi cho phép `ACTIVE`.
6. Tune wrist/waist trước, shoulder/elbow sau. Với shoulder/elbow, tune `gravity_bias` ở fixed pose
   trước, rồi mới tăng feedback fuzzy.

### D. Whole-arm fixed-pose

1. Dùng reference là sleep pose.
2. Tăng `u_max`, `Ke`, `Ked` và bias từng bước nhỏ; lưu rosbag/debug plots mỗi lần.
3. Chỉ thử step reference sau khi fixed-pose ổn định và soft/hard limits đã được test.
4. Không dùng tay đẩy trực tiếp arm ở giai đoạn đầu; dùng step nhỏ có kiểm soát.

### E. Dừng có kiểm soát

1. Đưa arm về pose thấp/an toàn khi controller còn ACTIVE.
2. Đỡ arm cơ khí.
3. Gọi `~/arm` với `data=false`; xác nhận command về zero.
4. Torque off hoặc dừng launch khi arm đã được đỡ.
5. Muốn quay lại position mode: vẫn giữ arm được đỡ, relaunch config mặc định rồi mới torque/command pose.

Không dùng `Ctrl+C` ở pose bất kỳ như một quy trình dừng an toàn được đảm bảo.

## 9. Tiêu chí hoàn thành MVP

MVP hoàn thành khi:

- Package build/test/install sạch và launch được từ `install/`.
- Không có thay đổi mới trong bốn repo vendor.
- Controller chỉ ACTIVE sau explicit arm và mode register readback thành công.
- Command rate bám theo mỗi mẫu `joint_states`, không chạy lặp trên dữ liệu cũ.
- Joint limits, stale-state check, output cap và slew limit đã được kích hoạt.
- Từng joint đã xác nhận đúng chiều PWM ở mức nhỏ.
- Arm giữ fixed sleep pose trong khoảng thời gian thử nghiệm đã định với sai số ghi nhận được.
- Có đồ thị error/velocity/PWM và biên bản model/register của phần cứng thật.
- Báo cáo ghi rõ giới hạn fail-safe đã nêu ở [Mục 4.5](#45-fault--disarm).

## 10. Hướng mở rộng sau MVP

### 10.1 Nâng baud

Chỉ nghiên cứu sau khi MVP 100 Hz ổn định. Lưu ý kỹ thuật: `Baud_Rate` trong `rx150.yaml` bị driver
bỏ qua khi load config (chỉ là ghi chú); baud host đang hardcode `DEFAULT_BAUDRATE=1000000`. Đổi baud
phải đổi **đồng bộ** tất cả motor và host, có backup/recovery bằng Dynamixel Wizard và benchmark packet
error. Không đổi baud trong cùng phiên thử nghiệm với thay đổi controller.

### 10.2 Fast Sync Read

Để đẩy tần số vòng lặp lên 150–300 Hz. Lưu ý kỹ thuật: `SyncReadHandler` lưu `GroupSyncRead*`, trong
khi `txPacket/rxPacket/txRxPacket` của DynamixelSDK **không virtual** và destructor lớp cha cũng không
virtual. Do đó muốn dùng Fast Sync Read phải có patch cấu trúc đúng kiểu ownership/call path, giới hạn
cho Protocol 2.0, build-test riêng và benchmark A/B rồi mới thử nâng tần số. 500 Hz không phải mặc định
hay tiêu chí bắt buộc.

### 10.3 Fail-safe command timeout

Hướng đúng: thêm watchdog tại process sở hữu Dynamixel bus — nếu quá hạn không nhận command mới thì
driver tự ghi Goal PWM = 0 hoặc torque off theo policy. Đây là thay đổi vendor/kiến trúc riêng, phải
thiết kế và kiểm thử fault injection trước khi gọi là fail-safe (xem giới hạn ở [Mục 4.5](#45-fault--disarm)).
