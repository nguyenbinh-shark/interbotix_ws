# Hướng Dẫn Chi Tiết & Báo Cáo Kỹ Thuật Dự Án: Điều Khiển Cánh Tay Robot Interbotix RX150 Bằng Bộ Điều Khiển Fuzzy Logic & MoveIt 2

---

## MỤC LỤC BÀI VIẾT BLOG HƯỚNG DẪN

1. **Tổng Quan Kiến Trúc & Triết Lý Thiết Kế**
2. **Quy Trình 1: Sinh Mã C Từ FIS & Trực Quan Hóa Mặt Điều Khiển 3D (Code Generation)**
3. **Quy Trình 2: Tuning An Toàn Từng Khớp Đơn (Single-Joint Safety Tuning & Overlay)**
4. **Quy Trình 3: Chạy Fuzzy Controller Toàn Arm & Live Tuning (GUI / CLI)**
5. **Quy Trình 4: Điều Khiển Hoạch Định Quỹ Đạo Bằng MoveIt 2 & Trajectory Bridge**
6. **Quy Trình 5: Giám Sát Realtime & Ghi Bag So Sánh A/B (PlotJuggler & Bag Recording)**
7. **Bảng Cheatsheet Tổng Hợp Lệnh Terminal**

---

## 1. TỔNG QUAN KIẾN TRÚC & TRIẾT LÝ THIẾT KẾ

### 1.1. Bối cảnh phần cứng & phần mềm
- **Robot**: Cánh tay robot 5 bậc tự do Interbotix RX150 dùng động cơ thông minh Dynamixel (XM540 / XM430, Protocol 2.0). Giao tiếp với PC thông qua adapter U2D2 (`/dev/ttyDXL`).
- **Môi trường**: ROS 2 Humble trên Ubuntu 22.04 LTS.
- **Triết lý không xâm nhập (Non-invasive design)**: Không sửa đổi mã nguồn gốc (upstream/vendor) của Interbotix. Mọi tính năng fuzzy và bridge được xây dựng biệt lập hoàn toàn trong gói ROS 2 package `fuzzy_controller`.

### 1.2. Tại sao lại dùng PWM mode thay vì Position mode mặc định?
- Mặc định, Dynamixel sử dụng bộ điều khiển PID vị trí **nằm sẵn trong firmware** của động cơ.
- Để bộ điều khiển **Fuzzy Logic (Mamdani Type-1)** do chúng ta lập trình trên máy chủ (host-side) trực tiếp điều khiển robot, ta chuyển các motor sang **`operating_mode: pwm`**. Lúc này, firmware chỉ đóng vai trò khuếch đại công suất, toàn bộ vòng kín vị trí do node ROS 2 C++ tính toán và gửi lệnh PWM raw (độ rộng xung).

### 1.3. Sơ đồ phân tầng hệ thống (3 Layer Architecture)
1. **Tầng Toán Fuzzy (C Pure)**: Mã nguồn `fuzzy_type1.c` / `fuzzy_type1.h` được sinh tự động từ file `.fis`. Không chứa mã ROS 2.
2. **Tầng Điều Khiển ROS 2 (C++ Node)**: Node `fuzzy_node` chạy timer (100 Hz hoặc 500 Hz), đọc sensor_msgs/JointState, gọi toán Fuzzy và publish `JointGroupCommand` (PWM).
3. **Tầng Giao Diện / Thượng Cấp**: MoveIt 2 (`move_group`), `fuzzy_trajectory_bridge` (Action Server), `fuzzy_gui` (Tkinter GUI), PlotJuggler.

---

## 2. QUY TRÌNH 1: SINH MÃ C TỪ FIS & TRỰC QUAN HÓA MẶT 3D

Mọi luật điều khiển Fuzzy bắt đầu từ file thiết kế `.fis` (MATLAB Fuzzy Inference System format).

### Bước 2.1: Chỉnh sửa thiết kế Fuzzy
File nguồn sự thật: `gen_fit_and_3d_graph/fuzzy_type1.fis` hoặc `src/fuzzy_controller/src/fuzzy/fuzzy_type1.fis`.
Định nghĩa:
- **Đầu vào 1 (`e`)**: Sai số vị trí $e \in [-1, 1]$ (đã chuẩn hóa qua $K_e$).
- **Đầu vào 2 (`ed`)**: Vận tốc sai số $\dot{e} \in [-1, 1]$ (đã chuẩn hóa qua $K_{ed}$).
- **Đầu ra (`u`)**: Lệnh điều khiển chuẩn hóa $u \in [-1, 1]$.
- **Phương pháp**: Mamdani (AND=min, OR=max, Implication=min, Defuzzification=centroid).

### Bước 2.2: Sinh mã C tự động bằng `fis2c.py`
Công cụ `fis2c.py` đọc `.fis` và tự động tạo mã C99 portable mà không cần sửa tay.

**Thực hiện trên Terminal:**
```bash
cd ~/interbotix_ws/gen_fit_and_3d_graph

# 1. Sinh file fuzzy_type1.h, fuzzy_type1.c và fuzzy_type1_demo.c
python3 fis2c.py fuzzy_type1.fis

# 2. Hoặc dùng script đồng bộ tự động vào ROS 2 package:
cd ~/interbotix_ws/src/fuzzy_controller/src/fuzzy
./regenerate.sh
```

### Bước 2.3: Tạo Web Artifact 3D xem mặt điều khiển (Control Surface)
Để kiểm tra tính liên tục và độ mượt của mặt điều khiển Fuzzy trước khi nạp lên robot real:

```bash
cd ~/interbotix_ws/gen_fit_and_3d_graph

# 1. Tính lưới 2D (e, ed) và xuất surface.json
python3 gen_surface.py

# 2. Đóng gói thành file HTML 3D tương tác standalone
python3 build_artifact.py

# 3. Mở file HTML trên trình duyệt
xdg-open fuzzy_surface.html
```
*Kết quả*: Mở trang web hiển thị khối 3D mặt điều khiển Fuzzy tương tác (xoay, thu phóng, xem contour).

---

## 3. QUY TRÌNH 2: TUNING AN TOÀN TỪNG KHỚP ĐƠN (`test_joint5.py` & `plot_runs.py`)

Thách thức lớn nhất khi điều khiển PWM là: nếu chọn sai Gain ($K_e, K_{ed}, K_u$), robot có thể bị vọt ngưỡng hoặc rung lắc nguy hiểm. Do đó dự án cung cấp quy trình **Kiểm thử độc lập Khớp 5 (`wrist_rotate`)**.

### Bước 3.1: Tại sao lại là Khớp 5?
- Khớp 5 (`wrist_rotate`) quay quanh trục dọc, không gánh trọng lực trực tiếp.
- Các khớp 1..4 vẫn giữ nguyên ở **Position mode** (firmware tự giữ vị trí bằng mô-men xoắn), chỉ riêng khớp 5 chuyển sang **PWM mode**.

### Bước 3.2: Chạy bài test Step Response khớp 5
Kịch bản test:
1. **Pha 1 (0 – 3s)**: Giữ vị trí hiện tại (`HOLD`).
2. **Pha 2 (3 – 16s)**: Bước nhảy đặt vị trí đích lệch $+2.0\text{ rad}$ ($\approx 115^\circ$).
3. **An toàn (Cleanup)**: Tự động đưa PWM về 0, chuyển khớp 5 về Position mode tại vị trí hiện tại (tránh giật nẩy khi kết thúc).

**Thực hiện trên Terminal:**
```bash
# Terminal 1: Chạy driver xs_sdk
source ~/interbotix_ws/install/setup.bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx150 use_rviz:=false

# Terminal 2: Biên dịch thư viện C tạm và chạy script test khớp 5
cd ~/interbotix_ws
gcc -shared -fPIC -O2 -o /tmp/fuzzy_type1.so gen_fit_and_3d_graph/fuzzy_type1.c -lm
python3 test_joint5.py
```
*Kết quả*: Script xuất log real-time và tự động tạo 2 file kết quả trong thư mục `runs/`:
- `runs/wrist_rotate_Ke0.2_Ked0.0005_Ku700.0_step+2.0_<timestamp>.csv`
- `runs/wrist_rotate_Ke0.2_Ked0.0005_Ku700.0_step+2.0_<timestamp>.png`

### Bước 3.3: So sánh A/B Tuning nhiều lần chạy qua `plot_runs.py`
Mỗi lần thay đổi tham số $K_e, K_{ed}, K_u$ trong `test_joint5.py` và chạy lại, file `.csv` mới được lưu vào `runs/`. Dùng `plot_runs.py` để chồng các đồ thị lên nhau so sánh đáp ứng quá độ.

```bash
cd ~/interbotix_ws

# Vẽ overlay tất cả các lần chạy trong runs/
python3 plot_runs.py

# Hoặc chỉ định các file CSV cụ thể
python3 plot_runs.py runs/run1.csv runs/run2.csv
```
*Đồ thị tạo ra (`runs/overlay.png`)* gồm 3 biểu đồ xếp chồng:
1. **Vị trí (pos vs ref)** theo thời gian.
2. **Điện áp PWM (u)** theo thời gian.
3. **Vận tốc (vel)** theo thời gian.

---

## 4. QUY TRÌNH 3: CHẠY FUZZY CONTROLLER TOÀN ARM & LIVE TUNING

Sau khi đã verified luật fuzzy an toàn trên khớp đơn, tiến hành chạy bộ điều khiển Fuzzy cho cả 5 khớp arm.

### Bước 4.1: Launch toàn bộ hệ thống Fuzzy Controller
Sử dụng file launch chính:
```bash
source ~/interbotix_ws/install/setup.bash
ros2 launch fuzzy_controller fuzzy_control.launch.py
```
*Tự động thực thi:*
1. Khởi động `xs_sdk` driver với file cấu hình động cơ `rx150_fuzzy.yaml`.
2. Khởi động `fuzzy_node` trong namespace `/rx150`.
3. Node gọi service `set_operating_modes` đưa cả 5 khớp arm sang **PWM mode**.
4. Bộ điều khiển bám pose mặc định (`reference_pose: [0.0, -1.80, 1.55, 0.8, 0.0]`).

### Bước 4.2: Điều khiển Setpoint & Tune Tham Số Bằng GUI Tkinter (`fuzzy_gui`)
Dự án có sẵn ứng dụng GUI Tkinter giao tiếp trực tiếp qua ROS 2 Topic & Parameter Service.

**Chạy GUI trên Terminal:**
```bash
source ~/interbotix_ws/install/setup.bash
ros2 run fuzzy_controller fuzzy_gui
```

**Tính năng trên GUI:**
- **Tab "Setpoint"**: 5 thanh trượt Slider tương ứng với 5 khớp (`waist`, `shoulder`, `elbow`, `wrist_angle`, `wrist_rotate`). Kéo trượt để publish trực tiếp tới topic `/rx150/fuzzy/setpoint`.
- **Tab "Gains"**: Bảng nhập tham số $K_e, K_{ed}, K_u, u_{max}$ cho từng khớp. Nút **Apply** gọi service `set_parameters` đè thông số tức thì vào vòng lặp điều khiển 100 Hz mà không cần khởi động lại Node.

### Bước 4.3: Tune tham số hoặc Đặt Setpoint qua ROS 2 CLI

**Đặt Setpoint trực tiếp bằng Command Line:**
```bash
# Gửi góc tham chiếu 5 khớp (đơn vị rad)
ros2 topic pub --once /rx150/fuzzy/setpoint std_msgs/msg/Float64MultiArray "{data: [0.0, -1.2, 1.0, 0.5, 0.0]}"
```

**Đổi Gain live qua ROS 2 Parameter Service:**
```bash
# Đổi Ke của 5 khớp
ros2 param set /rx150/fuzzy_node Ke "[2.0, 3.0, 3.0, 1.5, 2.0]"

# Đổi Ku (gain công suất PWM)
ros2 param set /rx150/fuzzy_node Ku "[600.0, 800.0, 700.0, 700.0, 700.0]"
```

---

## 5. QUY TRÌNH 4: ĐIỀU KHIỂN HOẠCH ĐỊNH QUỸ ĐẠO MOVEIT 2 & BRIDGE NODE

Để kết hợp khả năng né vật cản / quy hoạch động học ngược IK của **MoveIt 2** với bộ điều khiển **Fuzzy PWM**, dự án sử dụng node cầu nối `fuzzy_trajectory_bridge`.

### Bước 5.1: Nguyên lý hoạt động của Trajectory Bridge
- MoveIt 2 phát Action Goal dạng `control_msgs/action/FollowJointTrajectory`.
- `fuzzy_trajectory_bridge` lắng nghe Action Server `/rx150/arm_controller/follow_joint_trajectory`.
- Bridge chia nhỏ quỹ đạo TOTP (Time-Optimal Trajectory Parameterization) của MoveIt thành chuỗi các điểm vị trí theo mốc thời gian, sau đó **nội suy tuyến tính ở tần số 100 Hz** và đẩy vào topic `/rx150/fuzzy/setpoint`.
- `fuzzy_node` tiếp nhận setpoint liên tục và điều khiển động cơ bám theo bằng luồng PWM.

### Bước 5.2: Khởi động tích hợp MoveIt + Fuzzy Controller
Chạy file launch tích hợp duy nhất:

```bash
source ~/interbotix_ws/install/setup.bash

# Launch đầy đủ Driver + Fuzzy Node + Bridge + MoveGroup + RViz2
ros2 launch fuzzy_controller fuzzy_moveit.launch.py
```

### Bước 5.3: Thao tác điều khiển trên RViz 2
1. Khi RViz 2 mở ra, nhìn vào khung **MotionPlanning** ở góc trái.
2. Chọn tab **Planning**.
3. Kéo con trỏ End-Effector (cầu điều khiển màu cam/xanh) đến vị trí mong muốn trong không gian 3D, hoặc chọn sẵn pose ở menu `Goal State` (VD: `Home`, `Sleep`).
4. Nút **Plan**: MoveIt tính toán quỹ đạo không va chạm.
5. Nút **Execute** (hoặc **Plan & Execute**): Quỹ đạo được gửi xuống `fuzzy_trajectory_bridge` → Robot thực hiện chuyển động mượt mà bám theo bằng PWM Fuzzy.

---

## 6. QUY TRÌNH 5: GIÁM SÁT REALTIME & GHI BAG SO SÁNH A/B

Để phân tích chất lượng điều khiển (chỉ số overshoot, settling time, tracking error), dự án tích hợp PlotJuggler và script ghi dữ liệu ROS 2 bag.

### Bước 6.1: Mở PlotJuggler với Layout Nạp Sẵn

```bash
# Terminal 1: Chạy hệ thống robot
ros2 launch fuzzy_controller fuzzy_control.launch.py

# Terminal 2: Mở PlotJuggler
ros2 launch fuzzy_controller fuzzy_plot.launch.py
```

**Thao tác kết nối dữ liệu trong PlotJuggler:**
1. Chọn menu **Streaming** $\rightarrow$ **ROS2 Topic Subscriber** $\rightarrow$ Bấm **Add**.
2. Chọn các topic:
   - `/rx150/joint_states` (Vị trí & vận tốc thực tế)
   - `/rx150/fuzzy/reference` (Vị trí tham chiếu $q_{ref}$)
   - `/rx150/fuzzy/error` (Sai số vị trí $e$)
   - `/rx150/fuzzy/effort` (Xung PWM điều khiển $u$)
   - `/rx150/fuzzy/edot` (Vận tốc sai số $\dot{e}$)
3. Nút **Start** ($\blacktriangleright$): Các đồ thị trên 4 tab (Position, Velocity, PWM, Error) tự động hiển thị dữ liệu sóng real-time.

### Bước 6.2: Ghi Bag & So Sánh A/B (Chế độ Direct Step vs Ruckig Profile)
Node `fuzzy_node` hỗ trợ bộ phát sinh quỹ đạo mượt Ruckig OTG (`enable_profile: true`). Ta có thể so sánh giữa việc nhảy Step thô và chạy qua Ruckig Profile:

```bash
cd ~/interbotix_ws/src/fuzzy_controller

# Ghi lần 1: Đánh giá nhảy Step trực tiếp (bật config enable_profile: false)
./config/fuzzy_record.sh step_run

# Ghi lần 2: Đánh giá qua Ruckig Profile (bật config enable_profile: true)
./config/fuzzy_record.sh profile_run
```
*Cách xem lại*: Trong PlotJuggler $\rightarrow$ Menu `Data` $\rightarrow$ `Load ROS2 Bag` $\rightarrow$ Chọn cả 2 thư mục bag vừa ghi $\rightarrow$ Chồng 2 đường đồ thị để thấy sự khác biệt về độ mượt PWM và hiện tượng sụt áp/rung lắc.

---

## 7. BẢNG CHEATSHEET TỔNG HỢP LỆNH TERMINAL

| Công việc | Dòng lệnh Terminal |
|---|---|
| **Build Package** | `cd ~/interbotix_ws && colcon build --packages-select fuzzy_controller && source install/setup.bash` |
| **Sinh lại code C từ .fis** | `cd ~/interbotix_ws/src/fuzzy_controller/src/fuzzy && ./regenerate.sh` |
| **Xem mặt 3D Web** | `cd ~/interbotix_ws/gen_fit_and_3d_graph && python3 gen_surface.py && python3 build_artifact.py && xdg-open fuzzy_surface.html` |
| **Test an toàn Khớp 5** | `cd ~/interbotix_ws && gcc -shared -fPIC -O2 -o /tmp/fuzzy_type1.so gen_fit_and_3d_graph/fuzzy_type1.c -lm && python3 test_joint5.py` |
| **Vẽ đồ thị Overlay Runs** | `cd ~/interbotix_ws && python3 plot_runs.py` |
| **Launch Fuzzy Arm Real** | `ros2 launch fuzzy_controller fuzzy_control.launch.py` |
| **Mở GUI Tune Tkinter** | `ros2 run fuzzy_controller fuzzy_gui` |
| **Launch MoveIt 2 + Fuzzy** | `ros2 launch fuzzy_controller fuzzy_moveit.launch.py` |
| **Mở PlotJuggler Layout** | `ros2 launch fuzzy_controller fuzzy_plot.launch.py` |
| **Ghi ROS 2 Bag** | `cd ~/interbotix_ws/src/fuzzy_controller && ./config/fuzzy_record.sh my_test_bag` |
| **Set Parameter CLI** | `ros2 param set /rx150/fuzzy_node Ku "[600.0, 800.0, 700.0, 700.0, 700.0]"` |
| **Pub Setpoint CLI** | `ros2 topic pub --once /rx150/fuzzy/setpoint std_msgs/msg/Float64MultiArray "{data: [0.0, -1.8, 1.55, 0.8, 0.0]}"` |

---
*Báo cáo được tổng hợp đầy đủ cho việc soạn thảo bài viết blog kỹ thuật chi tiết.*
