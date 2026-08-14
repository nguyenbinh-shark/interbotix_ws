# Launch YOLO detector (+ tùy chọn hand_gesture node).
#
# Yêu cầu camera + TF đang chạy (từ fuzzy_moveit.launch.py use_camera:=true):
#   T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#           use_camera:=true use_camera_static_tf:=true use_handeye_publisher:=false
#   T2: ros2 launch rx150_perception yolo_detector.launch.py
#   (bật gesture): ros2 launch rx150_perception yolo_detector.launch.py enable_gesture:=true

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_weights = os.path.join(
        get_package_share_directory('rx150_perception'), 'models', 'best_color.pt')

    return LaunchDescription([
        DeclareLaunchArgument('weights_path', default_value=default_weights,
                              description='đường dẫn weights YOLO (.pt).'),
        DeclareLaunchArgument('confidence', default_value='0.5'),
        DeclareLaunchArgument('enable_roll', default_value='true',
                              description='tính yaw từ nắp cap (roll-from-cap).'),
        DeclareLaunchArgument('enable_gesture', default_value='false',
                              description='spawn thêm hand_gesture_node.'),

        Node(
            package='rx150_perception',
            executable='yolo_detector_node.py',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'weights_path': LaunchConfiguration('weights_path'),
                'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float),
                'enable_roll': ParameterValue(LaunchConfiguration('enable_roll'), value_type=bool),
            }],
        ),
        Node(
            package='rx150_perception',
            executable='hand_gesture_node.py',
            name='hand_gesture',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_gesture')),
        ),
    ])
