# Module tests

Thư mục này chứa các chương trình kiểm tra từng chức năng độc lập của hệ thống
RX150. File `COLCON_IGNORE` giúp `colcon` bỏ qua thư mục này khi dò ROS package.

## Cấu trúc

- `fuzzy_controller/`: kiểm tra luật mờ, gains, setpoint và trajectory bridge.
- `perception/`: kiểm tra camera, depth, YOLO, TF và xử lý point cloud.
- `moveit/`: kiểm tra planning, IK và thực thi trajectory.
- `hardware/`: kiểm tra riêng camera, motor, gripper và thiết bị ngoại vi.
- `common/`: mã tiện ích dùng chung cho nhiều bài test.

Mỗi bài test nên chỉ kiểm tra một chức năng và tự kiểm tra điều kiện đầu vào trước
khi tác động lên phần cứng. Không đặt model, rosbag hoặc log dung lượng lớn vào đây;
hãy đặt chúng trong `test_data/` và `test_results/` ở workspace root nếu cần.

## Chạy test

Sau khi build và source workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 module_tests/run_test.py --list
python3 module_tests/run_test.py perception/example_import_test.py
```

Xem camera và kết quả nhận diện YOLO trực tiếp:

```bash
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py
```

Mặc định bài test chỉ dùng ảnh màu. Để kiểm tra cả luồng depth:

```bash
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py -- --with-depth
```

Đổi model hoặc ngưỡng confidence:

```bash
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py -- \
  --weights src/rx150_perception/models/best_color.pt --confidence 0.35
```

Có thể truyền đối số cho bài test sau dấu `--`:

```bash
python3 module_tests/run_test.py hardware/my_motor_test.py -- --joint wrist_angle
```

## Quy ước file test

- Đặt tên `test_<chuc_nang>.py` hoặc `<chuc_nang>_test.py`.
- Test dùng robot thật phải có cảnh báo trong docstring và mặc định không phát lệnh.
- Trả mã thoát `0` khi thành công, khác `0` khi thất bại.
- Ghi rõ topic, service, action và phần cứng cần thiết ở đầu file.
