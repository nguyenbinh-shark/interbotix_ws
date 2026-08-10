
Bước 1 — xs_sdk đang chạy → có 

/rx150/joint_states
/rx150/xs_sdk

Bước 2 — joint_states đang chảy → có dữ liệu để vẽ

/rx150/joint_states: ~40 msg/s  ✓  (đang phát)

Bước 3 — PlotJuggler đã mở (pid 46681, load layout, 4 plugin ROS2 đã nạp, không segfault, không "Failed to launch")

plotjuggler -l .../fuzzy_plotjuggler_layout.xml  ✓

Phần GUI (bạn làm trên cửa sổ PlotJuggler vừa mở)
Bước 4 — Nhìn thanh bên trái, chọn tab Streaming.

Bước 5 — Trong danh sách streamer, click đúp (hoặc chọn rồi bấm) ROS2 Topic Subscriber. Một hộp thoại hiện ra.

Bước 6 — Trong hộp thoại: bấm Add → gõ /rx150/joint_states → Enter/OK. (PJ sẽ tự nhận diện kiểu sensor_msgs/JointState.)

Bước 7 — Bấm Start (nút tam giác xanh ▶) để bắt đầu stream.

Bước 8 — Qua tab Plots (bên phải) → các curve đã được layout sắp sẵn sẽ tự điền:

