# BSD 3-Clause License
# Copyright (c) hust
#
# Launch the Interbotix rx150 under xsarm_control and start the fuzzy controller.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Action 1 — bring up the xsarm driver/stack for the rx150, using our motor config.
    xsarm_launch = os.path.join(
        get_package_share_directory('interbotix_xsarm_control'),
        'launch',
        'xsarm_control.launch.py')

    motor_configs = os.path.join(
        get_package_share_directory('fuzzy_controller'),
        'config',
        'rx150_fuzzy.yaml')

    xsarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsarm_launch),
        launch_arguments={
            'robot_model': 'rx150',
            'robot_name': 'rx150',
            'motor_configs': motor_configs,
            'use_sim': 'false',
            'use_rviz': 'false',
        }.items())

    # Action 2 — fuzzy controller node (namespace 'rx150' -> relative topics become /rx150/...).
    fuzzy_params = os.path.join(
        get_package_share_directory('fuzzy_controller'),
        'config',
        'fuzzy_gains.yaml')

    fuzzy_node = Node(
        package='fuzzy_controller',
        executable='fuzzy_node',
        name='fuzzy_node',
        namespace='rx150',
        output='screen',
        parameters=[fuzzy_params])

    return LaunchDescription([xsarm, fuzzy_node])
