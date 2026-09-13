# -*- coding: utf-8 -*-
#
# 文件: dual_camera.launch.py
# 作用: 双相机启动文件(涵洞检测车左/右两台 MER-500-14GC)。
#
# 用参数文件 config/camera_params.yaml 启动 galaxy_camera_dual_node:
#   1) 按 cameras.left.* / cameras.right.* 寻址打开两台相机
#      (设备缺席时由节点内热插拔守望线程自动补开);
#   2) 每台相机一个独立采集线程, 各发布一条话题:
#        /left_camera/image_raw/compressed
#        /right_camera/image_raw/compressed
#
# 用法:
#   ros2 launch galaxy_camera_dual dual_camera.launch.py               # 按 yaml
#   ros2 launch galaxy_camera_dual dual_camera.launch.py sim:=false    # 真机
#   ros2 launch galaxy_camera_dual dual_camera.launch.py \
#       params_file:=/path/to/my_params.yaml
#
# 参数:
#   params_file  参数文件路径
#   sim          true/false 覆盖 yaml 里的 sim_mode; 缺省=尊重 yaml
#                (sim_mode=true 时用模拟设备顶替 SDK, 插拔语义与真机一致)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    from launch_ros.actions import Node

    params_file = LaunchConfiguration('params_file').perform(context)
    sim = LaunchConfiguration('sim').perform(context).strip().lower()

    # 只在显式传入 sim 时覆盖 yaml(空字符串不能赋给 bool 参数)
    overrides = {}
    if sim != '':
        overrides['sim_mode'] = sim in ('1', 'true', 'yes', 'on')

    return [Node(
        package='galaxy_camera_dual',
        executable='galaxy_camera_dual_node',
        name='galaxy_camera_dual',
        output='screen',
        emulate_tty=True,
        # 注意: 不用 respawn=True 自动重启(会与 Ctrl+C 关停竞争产生残留)。
        # 运行中掉线由节点内部的重连逻辑负责, 热插拔由守望线程负责。
        parameters=[params_file, overrides],
    )]


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('galaxy_camera_dual'),
        'config', 'camera_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file', default_value=params_file,
            description='相机参数文件路径'),
        DeclareLaunchArgument(
            name='sim', default_value='',
            description='true=模拟模式, false=真机; 缺省=用参数文件值'),
        OpaqueFunction(function=_setup),
    ])
