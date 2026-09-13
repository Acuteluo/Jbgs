# -*- coding: utf-8 -*-
#
# 文件: ir_camera.launch.py
# 作用: 启动红外热成像机芯驱动节点(唯一话题 /ir_camera/image_raw/compressed)。
#
# 用法:
#   ros2 launch ir_camera_driver ir_camera.launch.py              # 参数取 config/ir_camera_params.yaml
#   ros2 launch ir_camera_driver ir_camera.launch.py sim:=false   # 真机 UVC 模式
#   ros2 launch ir_camera_driver ir_camera.launch.py device_index:=2
#
# 参数:
#   params_file   参数文件路径(默认包内 config/ir_camera_params.yaml)
#   sim           true/false 覆盖 yaml 里的 sim_mode; 缺省=尊重 yaml
#   device_index  真机 V4L2 序号覆盖; 缺省=尊重 yaml
#
# 实现说明: sim/device_index 未传时不能把空字符串塞给 bool/int 参数
# (类型不符会导致节点启动失败), 因此用 OpaqueFunction 按需拼接参数。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    from launch_ros.actions import Node

    params_file = LaunchConfiguration('params_file').perform(context)
    sim = LaunchConfiguration('sim').perform(context).strip()
    device_index = LaunchConfiguration('device_index').perform(context).strip()

    # 只把"显式传入"的参数叠加到参数文件之上(顺序: 后者覆盖前者)
    overrides = {}
    if sim != '':
        overrides['sim_mode'] = sim.lower() in ('1', 'true', 'yes', 'on')
    if device_index != '' and device_index != '-1':
        overrides['device_index'] = int(device_index)

    node = Node(
        package='ir_camera_driver',
        executable='ir_camera_driver_node',
        name='ir_camera_driver',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, overrides],
    )
    return [node]


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('ir_camera_driver'),
        'config', 'ir_camera_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file', default_value=params_file,
            description='红外驱动参数文件路径'),
        DeclareLaunchArgument(
            name='sim', default_value='',
            description='true=模拟模式, false=真机UVC; 缺省=用参数文件值'),
        DeclareLaunchArgument(
            name='device_index', default_value='',
            description='真机 V4L2 设备序号; 缺省=用参数文件值'),
        OpaqueFunction(function=_setup),
    ])
