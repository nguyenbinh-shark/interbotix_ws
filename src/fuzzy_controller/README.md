# Fuzzy Controller cho Interbotix RX150 (ROS 2 Humble)

Package này cung cấp hệ thống điều khiển mờ (Fuzzy Logic Controller - Type 1 Mamdani) điều khiển trực tiếp mức xung (PWM mode) cho cánh tay robot Interbotix RX150 trên nền tảng ROS 2 Humble. 
Đặc biệt, hệ thống được thiết kế theo triết lý **không xâm nhập (Non-invasive)**: giữ nguyên mã nguồn gốc của Interbotix, tích hợp trơn tru với MoveIt 2 và hệ thống Perception (Camera 3D).

## Tính năng chính

1. **Điều khiển PWM vòng kín (Closed-loop PWM Control)**:
   - Thay thế bộ PID vị trí mặc định trong firmware của động cơ Dynamixel bằng bộ điều khiển Fuzzy tính toán trên máy tính chủ (host-side) chạy ở tần số cao (100 Hz).
2. **Biên dịch FIS sang C nguyên thuỷ**:
   - Luật điều khiển (Mamdani) được thiết kế từ file `.fis` (MATLAB format), sau đó được tự động sinh thành mã C thuần (`fuzzy_type1.c`) giúp tối ưu tốc độ thực thi, không phụ thuộc vào thư viện bên ngoài.
3. **Tích hợp MoveIt 2 (Trajectory Bridge)**:
   - Node cầu nối `fuzzy_trajectory_bridge` nội suy quỹ đạo an toàn từ MoveIt 2 (TOTP) và truyền thành các setpoint liên tục cho bộ Fuzzy.
4. **Tích hợp Nhận thức không gian 3D (Perception & Obstacle Avoidance)**:
   - Sẵn sàng cấu hình Octomap (`sensors_3d.yaml`) để đọc luồng PointCloud từ Camera 3D (ví dụ: Intel RealSense). MoveIt có thể tự động lập quỹ đạo lách qua các vật cản động.
5. **Giao diện Tune tham số trực tiếp (GUI)**:
   - Ứng dụng `fuzzy_gui` (Tkinter) cho phép tinh chỉnh Gains ($K_e, K_{ed}, K_u, u_{max}$) theo thời gian thực (Live-tuning) và lưu cấu hình trực tiếp vào YAML.
6. **Kiểm thử an toàn & Trực quan hoá**:
   - Cung cấp script test độc lập cho khớp 5 (không chịu tải) để tìm Gain an toàn.
   - Theo dõi dữ liệu vị trí, sai số, PWM tức thời thông qua PlotJuggler và hỗ trợ thu thập dữ liệu ROS 2 bag để so sánh A/B.

## Cấu trúc thư mục (Packages)

- `config/`: Chứa các file YAML cấu hình gain (`fuzzy_gains.yaml`), cấu hình MoveIt Perception (`sensors_3d.yaml`), cấu hình động cơ (`rx150_fuzzy.yaml`) và layout PlotJuggler.
- `launch/`: Các file khởi động tích hợp (chạy độc lập, chạy với MoveIt, chạy PlotJuggler).
- `scripts/`: Chứa mã nguồn Python cho GUI (`fuzzy_gui`), Node cầu nối quỹ đạo (`fuzzy_trajectory_bridge`) và kịch bản test (`fuzzy_bridge_test`).
- `src/`: Mã nguồn C++ Node chính (`fuzzy_node.cpp`) và thư viện mã C sinh từ FIS (`src/fuzzy/`).

## Hướng dẫn sử dụng nhanh

### 1. Khởi động với MoveIt 2 (Khuyên dùng)
Lệnh này khởi động toàn bộ: Driver xs_sdk, Fuzzy Node, Trajectory Bridge, MoveGroup, Static TF cho Camera và RViz 2.
```bash
ros2 launch fuzzy_controller fuzzy_moveit.launch.py
```
*(Nếu muốn test với Camera 3D để né vật cản, hãy chạy thêm node camera ở một terminal khác, ví dụ: `ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true`).*

### 2. Mở GUI tinh chỉnh thông số (Live Tuning)
```bash
ros2 run fuzzy_controller fuzzy_gui
```
- Sử dụng tab **Setpoint** để điều khiển robot thủ công.
- Sử dụng tab **Gains** để chỉnh sửa và **Lưu vào yaml** các thông số PID/Fuzzy.

### 3. Mở PlotJuggler theo dõi đồ thị
```bash
ros2 launch fuzzy_controller fuzzy_plot.launch.py
```
- Trên PlotJuggler, chọn Streaming -> ROS2 Topic Subscriber và Add các topic trong namespace `/rx150/fuzzy/` để xem đồ thị realtime.

## Thông tin chi tiết
Để xem hướng dẫn chuyên sâu về việc thiết kế luật mờ, cách sinh mã C từ FIS, kiểm thử an toàn từng khớp và so sánh hiệu suất qua bag file, vui lòng đọc tài liệu hướng dẫn kỹ thuật: [huong_dan_chi_tiet_blog.md](../../huong_dan_chi_tiet_blog.md).
