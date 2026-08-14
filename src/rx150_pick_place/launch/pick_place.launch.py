# Launch Layer 1 (YOLO + gesture) + Layer 2 (node quyết định pick-place MoveIt).
#
# Yêu cầu T1 đã chạy (motion stack + camera + hand-eye):
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true rs_camera_pointcloud_enable:=true \
#       use_camera_static_tf:=false use_handeye_publisher:=true
#
# Chạy T2 (cái này):
#   ros2 launch rx150_pick_place pick_place.launch.py
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pp_share = get_package_share_directory('rx150_pick_place')
    perc_share = get_package_share_directory('rx150_perception')
    params_file = os.path.join(pp_share, 'config', 'pick_place_params.yaml')

    return LaunchDescription([
        # ---- Layer 1: YOLO detector + hand_gesture (gesture BẬT để chọn vật) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perc_share, 'launch', 'yolo_detector.launch.py')),
            launch_arguments={'enable_gesture': 'true'}.items(),
        ),
        # ---- Layer 2: node quyết định pick-place qua MoveIt ----
        Node(
            package='rx150_pick_place',
            executable='pick_place_moveit_node.py',
            name='pick_place_moveit',
            output='screen',
            parameters=[params_file],
        ),
    ])
