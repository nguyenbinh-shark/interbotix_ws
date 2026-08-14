"""Mở PlotJuggler với layout sẵn cho fuzzy controller (attach vào session robot đang chạy).

Cách dùng (2 terminal):

  # Terminal 1 — chạy robot + bộ fuzzy
  ros2 launch fuzzy_controller fuzzy_control.launch.py

  # Terminal 2 — mở PlotJuggler (cần đã cài: sudo apt install ros-humble-plotjuggler-ros)
  ros2 launch fuzzy_controller fuzzy_plot.launch.py

Sau khi PlotJuggler mở, plugin ROS2 KHÔNG tự subscribe qua CLI, phải bật tay:
  Menu Streaming -> ROS2 Topic Subscriber -> Add các topic:
    /rx150/joint_states
    /rx150/fuzzy/reference
    /rx150/fuzzy/effort
    /rx150/fuzzy/error
    /rx150/fuzzy/edot
  Khi data chảy, các plot trong layout tự điền (4 tab: Position/Velocity/PWM/Error, 5 khớp/tab).

So sánh A/B (step vs profile): record 2 bag bằng config/fuzzy_record.sh, rồi trong PlotJuggler
vào Data -> Load -> ROS2 bag mở cả hai, overlay cùng 1 curve.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    layout = os.path.join(
        get_package_share_directory("fuzzy_controller"),
        "config",
        "fuzzy_plotjuggler_layout.xml",
    )
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "run", "plotjuggler", "plotjuggler", "-l", layout],
                name="plotjuggler",
                output="screen",
            )
        ]
    )
