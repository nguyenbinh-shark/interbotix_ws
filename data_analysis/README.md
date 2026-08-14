# Data Analysis — Công cụ phân tích & trực quan hoá dữ liệu RX150

Module **dùng chung** cho toàn bộ dự án, phục vụ thu thập, trực quan hoá realtime
và phân tích offline dữ liệu điều khiển. Khi thêm bộ điều khiển mới (PID, MPC, ...),
chỉ cần thêm layout XML vào `layouts/` — không cần sửa code.

## Cấu trúc

```
data_analysis/
├── README.md
├── requirements.txt
├── csv_logger.py              # ROS 2 node: subscribe topic → ghi CSV
├── plot_control_csv.py        # Script Python thuần: vẽ đồ thị từ CSV
├── layouts/                   # PlotJuggler layouts (realtime monitoring)
│   └── fuzzy_plotjuggler_layout.xml
└── utils/
    ├── __init__.py
    ├── csv_loader.py          # Load & validate CSV, auto-detect khớp
    └── plot_styles.py         # Màu sắc, style, cấu hình chung
```

## 1. Trực quan hoá Realtime — PlotJuggler

PlotJuggler (`ros-humble-plotjuggler-ros`) subscribe trực tiếp vào ROS 2 topic,
hiển thị đồ thị realtime khi robot đang chạy.

```bash
# Cài đặt (1 lần)
sudo apt install ros-humble-plotjuggler-ros

# Mở PlotJuggler với layout cho fuzzy controller
ros2 run plotjuggler plotjuggler -l ~/interbotix_ws/data_analysis/layouts/fuzzy_plotjuggler_layout.xml
```

**Thêm layout cho controller mới:** Tạo file XML trong `layouts/`, ví dụ `pid_plotjuggler_layout.xml`.

Sau khi PlotJuggler mở:
1. Chọn menu **Streaming** → **ROS2 Topic Subscriber** → **Add** các topic
2. Layout sẽ tự điền data vào các đồ thị đã cấu hình sẵn

## 2. Thu thập dữ liệu — CSV Logger

```bash
# Ghi data fuzzy (mặc định)
python3 csv_logger.py --ros-args -p controller_prefix:=fuzzy

# Ghi data controller khác
python3 csv_logger.py --ros-args -p controller_prefix:=pid

# Ctrl+C để dừng → file CSV sẵn sàng
```

## 3. Phân tích Offline — Plot CSV

```bash
# Vẽ đồ thị từ CSV (tạo bởi csv_logger hoặc bất kỳ controller nào)
python3 plot_control_csv.py fuzzy_data_xxx.csv

# Chỉ vẽ 2 khớp
python3 plot_control_csv.py data.csv --joints waist shoulder

# Lưu hình cho báo cáo/paper
python3 plot_control_csv.py data.csv --save result.png

# Cắt theo thời gian
python3 plot_control_csv.py data.csv --start 2.0 --end 10.0
```

## Cài đặt (phần Python)

```bash
pip install -r requirements.txt
```

## Tương thích

Các tool hoạt động với bất kỳ bộ điều khiển nào, miễn tuân theo quy ước:

**CSV format** — cột `timestamp` (bắt buộc) + các cột theo pattern:
- `{joint}_pos`, `{joint}_vel` — vị trí, vận tốc thực
- `{joint}_err` — sai số
- `{joint}_edot` — đạo hàm sai số
- `{joint}_pwm` — tín hiệu điều khiển
- `{joint}_grav` — bù trọng lực
- `{joint}_ref_pos`, `{joint}_ref_vel` — reference (tuỳ chọn)

**ROS 2 topic** — publish dưới namespace `/{robot}/{prefix}/`:
- `/{robot}/{prefix}/error` → `JointState.position`
- `/{robot}/{prefix}/effort` → `JointState.effort`
- `/{robot}/{prefix}/edot` → `JointState.velocity`
- `/{robot}/{prefix}/gravity` → `JointState.effort`
- `/{robot}/{prefix}/reference` → `JointState.position` + `velocity`
