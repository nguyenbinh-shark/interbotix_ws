# Data Analysis — Công cụ Thu thập, Giám sát & Phân tích Dữ liệu Điều khiển RX150

Module **dùng chung** cho toàn bộ hệ thống cánh tay robot Interbotix RX150. Phục vụ thu thập dữ liệu (logging), giám sát trực quan thời gian thực (real-time monitoring) và vẽ đồ thị phân tích offline phục vụ báo cáo/bài báo khoa học.

---

## 📂 Cấu trúc Module

```
data_analysis/
├── README.md                      # Tài liệu hướng dẫn sử dụng chi tiết này
├── requirements.txt               # Các thư viện Python cần thiết (matplotlib, pandas, numpy...)
├── csv_logger.py                  # ROS 2 Node: Subscribe topics → Ghi dữ liệu ra CSV tự động
├── plot_control_csv.py            # Python Script: Vẽ đồ thị phân tích chất lượng cao từ CSV
├── layouts/                       # Danh mục PlotJuggler XML Layouts
│   └── fuzzy_plotjuggler_layout.xml   # Layout chuẩn hoá cho Fuzzy / PID Controller (4 tabs)
└── utils/
    ├── __init__.py
    ├── csv_loader.py              # Đọc & kiểm tra định dạng CSV, tự động nhận diện danh sách khớp
    └── plot_styles.py             # Cấu hình phong cách đồ thị (chuẩn IEEE/MDPI, màu sắc, font)
```

---

## 📈 1. Giám sát & Trực quan hoá dữ liệu với PlotJuggler

[PlotJuggler](https://github.com/facontidavide/PlotJuggler) là công cụ đồ hoạ GUI mạnh mẽ giúp hiển thị dữ liệu sóng (time-series) theo thời gian thực hoặc xem lại các bản ghi thí nghiệm.

### 1.1. Cài đặt (1 lần duy nhất)

```bash
sudo apt update
sudo apt install ros-humble-plotjuggler-ros
```

---

### 1.2. Khởi chạy PlotJuggler với Layout cấu hình sẵn

Layout XML đi kèm (`layouts/fuzzy_plotjuggler_layout.xml`) đã được chia sẵn làm **4 tab chuyên biệt**:
1. **Position & Reference**: Đồ thị vị trí thực ($q$) và tham chiếu ($q_{ref}$) cho từng khớp.
2. **Joint Velocity**: Đồ thị vận tốc góc thực ($\dot{q}$) và vận tốc tham chiếu ($\dot{q}_{ref}$).
3. **Error & Edot**: Sai số vị trí góc ($e$) và đạo hàm sai số ($\dot{e}$).
4. **Control Effort (PWM)**: Tín hiệu điều khiển PWM ($u$) gửi tới các động cơ Dynamixel.

**Lệnh khởi chạy:**
```bash
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler -l ~/interbotix_ws/data_analysis/layouts/fuzzy_plotjuggler_layout.xml
```

---

### 1.3. Quy trình Giám sát Realtime (Live Streaming)

> [!NOTE]
> Yêu cầu hệ thống robot hoặc mô phỏng (Gazebo/rviz) đang chạy lệnh `launch` điều khiển trước.

1. **Khởi động Streaming Plugin**:
   - Trên menu PlotJuggler, chọn bảng điều khiển **Streaming** (ở cột bên trái).
   - Trong dropdown menu, chọn **ROS2 Topic Subscriber**.
   - Bấm nút **Start** ($\blacktriangleright$).

2. **Chọn ROS 2 Topics để thu thập**:
   - Hộp thoại danh sách topic hiện lên, chọn các topic cần xem:
     - `/{robot}/{prefix}/reference` (VD: `/rx150/fuzzy/reference`)
     - `/{robot}/{prefix}/error` (VD: `/rx150/fuzzy/error`)
     - `/{robot}/{prefix}/effort` (VD: `/rx150/fuzzy/effort`)
     - `/{robot}/{prefix}/edot` (VD: `/rx150/fuzzy/edot`)
     - `/{robot}/joint_states` (Vị trí & vận tốc góc thực tế)
   - Bấm **OK**.

3. **Thao tác trên đồ thị Realtime**:
   - **Tạm dừng/Tiếp tục (Pause/Resume)**: Bấm space hoặc nút Pause trên thanh công cụ để dừng màn hình kiểm tra điểm dị thường.
   - **Buffer Time**: Thay đổi độ dài thời gian hiển thị (mặc định 30 giây) ở góc dưới giao diện.
   - **Zoom / Pan**: Cuộn chuột để phóng to/thu nhỏ, giữ chuột phải để di chuyển khung nhìn.

---

### 1.4. Quy trình Phân tích Offline & So sánh A/B (ROS Bag & CSV)

PlotJuggler hỗ trợ nạp đồng thời nhiều tệp dữ liệu để **chồng đồ thị (overlay)**, giúp so sánh đáp ứng giữa các bộ điều khiển (ví dụ: Fuzzy vs PID) hoặc các kịch bản quỹ đạo (Step vs Profile).

1. **Nạp bản ghi ROS 2 Bag (`.db3` / `.mcap`)**:
   - Vào menu **Data** $\rightarrow$ **Load Data from File**.
   - Chọn định dạng **ROS2 storage plugin** (`.db3` hoặc `.mcap`) trong thư mục dữ liệu (VD: `src/rx150_fuzzy_controller/data/step_.../`).
   - Chọn các topic cần nạp $\rightarrow$ Bấm **OK**.

2. **Nạp thêm bản ghi thứ 2 để so sánh (Overlay)**:
   - Tiếp tục chọn **Data** $\rightarrow$ **Load Data from File** $\rightarrow$ Chọn bản ghi khác (VD: file bag của bộ điều khiển PID).
   - Chọn **Prefix** tên dữ liệu để phân biệt hai đợt chạy (VD: `fuzzy_` và `pid_`).
   - Kéo thả đường đặc tính của đợt 2 vào cùng khung đồ thị với đợt 1 để so sánh sai số xác lập ($e_{ss}$), thời gian quá độ ($t_s$), độ vọt lố (%OS).

3. **Nạp dữ liệu từ file CSV**:
   - Chọn **Data** $\rightarrow$ **Load Data from File** $\rightarrow$ Đổi bộ lọc file thành `CSV (*.csv)`.
   - Chọn cột `timestamp` làm trục thời gian $X$.

---

### 1.5. Xuất dữ liệu CSV từ PlotJuggler GUI

Nếu bạn muốn trích xuất dữ liệu của một khoảng thời gian cụ thể từ đồ thị đang xem ra CSV:
1. Vào menu **Toolbox** $\rightarrow$ chọn **CSV Exporter**.
2. Tích chọn các chuỗi dữ liệu (curves) cần xuất.
3. Chọn các tùy chọn:
   - **Time Range**: Export toàn bộ hoặc chỉ vùng đang Zoom trên màn hình.
   - **Resample**: Đồng bộ tần số lấy mẫu (nếu các topic có tần số phát khác nhau).
4. Bấm **Export to File** $\rightarrow$ Chọn thư mục lưu `.csv`.

---

### 1.6. Tạo & Tùy biến XML Layout mới

Khi thêm bộ điều khiển mới (như MPC hay Sliding Mode), bạn có thể lưu layout riêng:
1. Sắp xếp các ô đồ thị và kéo thả các biến tương ứng từ cột **Timeseries List** vào màn hình.
2. Đổi màu sắc: Click chuột phải vào tên biến ở góc đồ thị $\rightarrow$ **Change Color**.
3. Lưu layout: Vào **File** $\rightarrow$ **Save Layout to File** $\rightarrow$ Lưu vào thư mục `data_analysis/layouts/pid_plotjuggler_layout.xml`.

---

## 📝 2. Thu thập dữ liệu tự động — `csv_logger.py`

Node Python `csv_logger.py` giúp tự động ghi lại toàn bộ dữ liệu điều khiển ra file CSV trong quá trình thử nghiệm mà không cần thao tác thủ công trên GUI.

### 2.1. Lệnh thực thi

```bash
cd ~/interbotix_ws/data_analysis

# Ghi dữ liệu bộ điều khiển Fuzzy (mặc định)
python3 csv_logger.py --ros-args -p controller_prefix:=fuzzy -p robot_name:=rx150

# Ghi dữ liệu bộ điều khiển PID
python3 csv_logger.py --ros-args -p controller_prefix:=pid -p robot_name:=rx150
```

> [!TIP]
> Nhấn `Ctrl + C` để kết thúc phiên ghi. File CSV sẽ tự động được tạo tại thư mục hiện hành với định dạng tên chứa nhãn thời gian: `fuzzy_data_YYYYMMDD_HHMMSS.csv`.

---

## 📊 3. Phân tích Offline & Vẽ đồ thị Báo cáo — `plot_control_csv.py`

Script `plot_control_csv.py` đọc file CSV dữ liệu thí nghiệm và tự động xuất ra các đồ thị đạt chuẩn công bố khoa học (IEEE/MDPI) với độ phân giải cao (300 DPI).

### 3.1. Các lệnh phổ biến

```bash
cd ~/interbotix_ws/data_analysis

# 1. Vẽ đồ thị mặc định cho tất cả các khớp
python3 plot_control_csv.py fuzzy_data_20260813_140000.csv

# 2. Chỉ vẽ 2 khớp quan tâm (waist & shoulder)
python3 plot_control_csv.py fuzzy_data_20260813_140000.csv --joints waist shoulder

# 3. Cắt khoảng thời gian phân tích (từ giây thứ 2 đến giây thứ 10)
python3 plot_control_csv.py fuzzy_data_20260813_140000.csv --start 2.0 --end 10.0

# 4. Xuất ảnh phân giải cao cho báo cáo / bài báo (PNG / PDF / SVG)
python3 plot_control_csv.py fuzzy_data_20260813_140000.csv --save report_fuzzy_response.png
```

---

## 🔗 4. Quy chuẩn Chuẩn hoá dữ liệu (Topic & CSV Standard Mapping)

Để hệ thống hoạt động dạng **Plug-and-Play** với bất kỳ bộ điều khiển nào, toàn bộ các gói trong workspace cần tuân thủ quy chuẩn đặt tên sau:

### Cấu trúc cột File CSV:
| Tên cột | Ý nghĩa kỹ thuật | Đơn vị |
| :--- | :--- | :--- |
| `timestamp` | Thời gian tính từ khi bắt đầu ghi dữ liệu | Giây ($s$) |
| `{joint}_pos` | Vị trí góc thực tế của khớp | Radian ($rad$) |
| `{joint}_vel` | Vận tốc góc thực tế của khớp | Rad/s |
| `{joint}_ref_pos` | Vị trí góc đặt / tham chiếu ($q_{ref}$) | Radian ($rad$) |
| `{joint}_ref_vel` | Vận tốc góc đặt / tham chiếu ($\dot{q}_{ref}$) | Rad/s |
| `{joint}_err` | Sai số vị trí góc ($e = q_{ref} - q$) | Radian ($rad$) |
| `{joint}_edot` | Đạo hàm sai số ($\dot{e} = \dot{q}_{ref} - \dot{q}$) | Rad/s |
| `{joint}_pwm` | Tín hiệu điều khiển xung PWM | $-1023 \dots 1023$ |
| `{joint}_grav` | Thành phần Momen / PWM bù trọng lực | Xung PWM |

### ROS 2 Topics chuẩn:
- `/{robot}/{prefix}/reference` (`sensor_msgs/msg/JointState`)
- `/{robot}/{prefix}/error` (`sensor_msgs/msg/JointState`)
- `/{robot}/{prefix}/edot` (`sensor_msgs/msg/JointState`)
- `/{robot}/{prefix}/effort` (`sensor_msgs/msg/JointState`)
- `/{robot}/joint_states` (`sensor_msgs/msg/JointState`)
