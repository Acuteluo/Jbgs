# -*- coding: utf-8 -*-
#
# 文件: all.launch.py
# 作用: ★ 总启动: 双大恒 + 红外 + 传感器 + core_node(同屏) + 巡检协议 ★
#
# 用法(推荐经工程根 run.sh 启动, 它会先构建并导出 JBGS_ROOT):
#   ./run.sh                          # 干净重建 + 启动全部
#   ros2 launch bringup all.launch.py # 直接启动(需先 source install)
#
# 常用 launch 参数(全部可空=尊重 config/launch.json, 详见 launch_common):
#   sim:=false            真机模式
#   show_windows:=false   无头模式(测试/无图形环境)
#   in_trulyworking:=false 关闭巡检协议
#   left_absent:=20       左大恒 20 秒后才"接入"(热插拔测试)
#   left_disc_at:=30 left_disc_dur:=5   左大恒 30s 起断连 5 秒
#
# 启动后话题:
#   /left_camera/image_raw/compressed   左大恒 JPEG
#   /right_camera/image_raw/compressed  右大恒 JPEG
#   /ir_camera/image_raw/compressed     红外 JPEG
#   /sensor/env                         温湿度/CO2
#   /core_node/status                   1Hz 心跳
#   /inspection/command(入, UInt8 0x01) /inspection/ack(出, UInt8 0x02)
#   /inspection_node/status             巡检状态机状态(10Hz)

import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launch_common as common   # noqa: E402


def _setup(context, *args, **kwargs):
    """集中收集全部 launch 参数后交给 launch_common 组装节点。"""

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    names = ('sim', 'show_windows', 'in_trulyworking', 'sensor',
             'input_topic', 'ack_topic', 'save_dir', 'frame_timeout',
             'yolo', 'ir_seepage',
             'absent', 'disc_at', 'disc_dur',
             'left_absent', 'right_absent',
             'left_disc_at', 'left_disc_dur',
             'right_disc_at', 'right_disc_dur',
             'ir_absent', 'ir_disc_at', 'ir_disc_dur',
             'sensor_absent', 'sensor_disc_at', 'sensor_disc_dur')
    args = {n: arg(n) for n in names}

    # 日志级别(可选): 以 --log-level 附加到每个节点。
    level = common.log_level_arg(args)
    if level not in ('debug', 'info', 'warn', 'error'):
        level = ''

    actions = [
        common.galaxy_node(args, level),        # 双大恒
        common.ir_node(args, level),            # 红外
        common.core_node(args, level),          # 同屏显示 + 检测
        common.inspection_node(args, level),    # 巡检协议
    ]
    sensor_flag = args['sensor'].strip().lower()
    if sensor_flag == '':
        enabled = bool(common.load_launch_config().get('enable_sensor', True))
    else:
        enabled = common.truthy(sensor_flag)
    if enabled:
        actions.append(common.sensor_node(args, level))
    return actions


def generate_launch_description():
    common.start_key_forward_if_tty()
    return LaunchDescription([
        *common.standard_arguments(),
        OpaqueFunction(function=_setup),
    ])
