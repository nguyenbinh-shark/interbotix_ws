# Data Analysis — Công cụ phân tích dữ liệu điều khiển RX150

Module Python **độc lập** (không phụ thuộc ROS 2) để phân tích và vẽ đồ thị
từ file CSV do các bộ điều khiển tạo ra.

## Cấu trúc

```
data_analysis/
├── README.md
├── requirements.txt
├── plot_control_csv.py      # Script chính: vẽ đồ thị từ CSV
└── utils/
    ├── __init__.py
    ├── csv_loader.py        # Load & validate CSV data
    └── plot_styles.py       # Màu sắc, style, cấu hình chung
```

## Cài đặt

```bash
pip install -r requirements.txt
```

## Cách dùng

```bash
# Vẽ đồ thị từ CSV (tạo bởi fuzzy_csv_logger hoặc bất kỳ controller nào)
python3 plot_control_csv.py fuzzy_data_xxx.csv

# Chỉ vẽ 2 khớp
python3 plot_control_csv.py data.csv --joints waist shoulder

# Lưu hình
python3 plot_control_csv.py data.csv --save result.png

# Cắt theo thời gian
python3 plot_control_csv.py data.csv --start 2.0 --end 10.0
```

## Tương thích

Script hoạt động với bất kỳ file CSV nào có cấu trúc cột:
- `timestamp` (bắt buộc)
- `{joint}_pos`, `{joint}_vel` — vị trí, vận tốc thực
- `{joint}_err` — sai số
- `{joint}_edot` — đạo hàm sai số
- `{joint}_pwm` — tín hiệu điều khiển
- `{joint}_grav` — bù trọng lực
- `{joint}_ref_pos`, `{joint}_ref_vel` — reference (tuỳ chọn)

Khi thay bộ điều khiển (PID, MPC, ...), chỉ cần đảm bảo CSV logger
xuất ra đúng format trên → script plot dùng lại được ngay.
