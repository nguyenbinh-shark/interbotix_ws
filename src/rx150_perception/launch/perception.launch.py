# Pipeline nhận diện PCL: include interbotix_perception_modules/pc_filter.launch.py
# sub `/camera/camera/depth/color/points`, ra cluster centroid theo ref_frame.
#
# Chạy (camera + TF phải đang chạy từ fuzzy_moveit.launch.py):
#   ros2 launch rx150_perception perception.launch.py
# Bật pipeline liên tục (để tune filter / xem RViz):
#   ros2 launch rx150_perception perception.launch.py enable_pipeline:=true use_pointcloud_tuner_gui:=true
# Lấy cluster theo yêu cầu (tiết kiệm CPU, dùng cho pick-place):
#   ros2 service call /pc_filter/enable_pipeline std_srvs/srv/SetBool "{data: true}"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pc_filter_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_perception_modules'),
            'launch', 'pc_filter.launch.py',
        ])),
        launch_arguments={
            'filter_ns': LaunchConfiguration('filter_ns'),
            'filter_params': LaunchConfiguration('filter_params'),
            'enable_pipeline': LaunchConfiguration('enable_pipeline'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'use_pointcloud_tuner_gui': LaunchConfiguration('use_pointcloud_tuner_gui'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'filter_ns', default_value='pc_filter',
            description='namespace cho pointcloud_pipeline (topics/services nằm dưới đây).'),
        DeclareLaunchArgument(
            'filter_params', default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'config', 'filter_params.yaml',
            ]),
            description='file filter params (voxel/crop/plane/cluster).'),
        DeclareLaunchArgument(
            'enable_pipeline', default_value='false',
            description='true = chạy liên tục; false = chỉ chạy khi gọi get_cluster_positions.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/camera/camera/depth/color/points',
            description='topic pointcloud đầu vào.'),
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui', default_value='false',
            description='hiện GUI tune filter online.'),
        pc_filter_include,
    ])
