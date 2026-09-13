# -*- coding: utf-8 -*-
#
# 文件: galaxy_camera.launch.py
# 作用: 单相机启动文件(只用 left 组一台; 话题仍为 /left_camera/image_raw/...)
#
# 与 dual_camera.launch.py 的区别仅是参数文件:
#   - 使用 config/camera_params_single.yaml: 相机组只有 [left] 一个;
#   - 话题统一为 /left_camera/image_raw/compressed,
#     core_node 与显示端无需区分单/双相机模式。
#
# 用法:
#   ros2 launch galaxy_camera_dual galaxy_camera.launch.py             # 按 yaml
#   ros2 launch galaxy_camera_dual galaxy_camera.launch.py sim:=false  # 真机

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    from launch_ros.actions import Node

    params_file = LaunchConfiguration('params_file').perform(context)
    sim = LaunchConfiguration('sim').perform(context).strip().lower()

    overrides = {}
    if sim != '':
        overrides['sim_mode'] = sim in ('1', 'true', 'yes', 'on')

    return [Node(
        package='galaxy_camera_dual',
        executable='galaxy_camera_dual_node',
        name='galaxy_camera',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, overrides],
    )]


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('galaxy_camera_dual'),
        'config', 'camera_params_single.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file', default_value=params_file,
            description='相机参数文件路径'),
        DeclareLaunchArgument(
            name='sim', default_value='',
            description='true=模拟模式, false=真机; 缺省=用参数文件值'),
        OpaqueFunction(function=_setup),
    ])
