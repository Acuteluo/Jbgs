# -*- coding: utf-8 -*-
#
# 文件: sensor_driver.launch.py
# 作用: 独立启动塔石温湿度二氧化碳传感器驱动节点。
#
# 用法:
#   ros2 launch tas_sensor_driver sensor_driver.launch.py
#
# 启动后:
#   - 话题 /sensor/env   : 每 1s 发布一次环境数据(温度/湿度/CO2)
#   - 服务 /sensor/query : 按需查询一次, 立即返回最新数据

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 参数文件默认路径
    params_file = os.path.join(
        get_package_share_directory('tas_sensor_driver'),
        'config', 'sensor_driver.yaml')

    return LaunchDescription([
        # 允许命令行覆盖参数文件
        DeclareLaunchArgument(name='params_file', default_value=params_file),

        Node(
            package='tas_sensor_driver',           # 所属功能包
            executable='tas_sensor_driver_node',   # 可执行文件名
            name='tas_sensor_driver',              # 节点名
            output='screen',                       # 日志直接打到终端
            emulate_tty=True,                      # 彩色日志
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
