# Fuzzy Codegen & 3D Surface Visualizer — Công cụ sinh mã C & Trực quan hoá Mặt Mờ

Module chứa các công cụ tự động biên dịch thiết kế luật mờ từ MATLAB (`.fis`) sang mã C thuần (`.c`/`.h`) tối ưu tốc độ thực thi, và tạo trang web trực quan hoá mặt 3D mờ (`fuzzy_surface.html`).

---

## 📂 Các tệp trong thư mục

- `fis2c.py`: Script Python dịch file `.fis` (MATLAB FIS) sang mã C (`fuzzy_type1.c`, `fuzzy_type1.h`).
- `gen_surface.py`: Script tính toán lưới điểm 3D mặt mờ và lưu thành `surface.json`.
- `build_surface_html.py`: Gộp dữ liệu `surface.json` vào giao diện web tự chứa `fuzzy_surface.html`.
- `fuzzy_type1.fis`: File gốc luật mờ thiết kế.
- `fuzzy_type1.c` / `fuzzy_type1.h`: Mã C nguyên thuỷ được sinh ra để tích hợp vào C++ ROS 2 Node (`rx150_fuzzy_controller`).
- `fuzzy_type1_demo.c`: File demo test mã C độc lập với `gcc`.

---

## 🚀 Hướng dẫn sử dụng

### 1. Sinh lại mã C từ file thiết kế `.fis`:
```bash
cd ~/interbotix_ws/fuzzy_codegen
python3 fis2c.py fuzzy_type1.fis
```

### 2. Sinh mặt đáp ứng 3D (3D Response Surface) cho Web:
```bash
cd ~/interbotix_ws/fuzzy_codegen
python3 gen_surface.py && python3 build_surface_html.py
xdg-open fuzzy_surface.html
```
