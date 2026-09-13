# -*- coding: utf-8 -*-
#
# 文件: launch_common.py
# 作用: bringup 总启动(all.launch.py)的节点组装与配置装载(单一来源)。
#
# 配置优先级(高 -> 低):
#   1. launch 参数(sim:= / show_windows:= / left_absent:= ...);
#   2. 工程根目录 config/launch.json(总配置);
#   3. 各驱动包自带 yaml 默认值。
#
# 工程根目录定位: 环境变量 JBGS_ROOT(run.sh 导出)优先;
# 否则从当前工作目录向上搜索含 config/launch.json 的目录。
# 相机设备 ID/别名/内参占位读取 config/galaxy_camera1_.json、
# config/galaxy_camera_2.json、config/infrared_camera.json。

import json
import os

from ament_index_python.packages import get_package_share_directory


# ----------------------------------------------------------------------
# 工程根目录与总配置
# ----------------------------------------------------------------------

def find_project_root():
    """定位工程根目录(含 config/launch.json); 找不到返回 None。"""
    env = os.environ.get('JBGS_ROOT', '').strip()
    if env and os.path.isfile(os.path.join(env, 'config', 'launch.json')):
        return os.path.abspath(env)
    cur = os.getcwd()
    for _ in range(6):   # 向上最多 6 层
        if os.path.isfile(os.path.join(cur, 'config', 'launch.json')):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return None


_PROJECT_ROOT = find_project_root()


def load_launch_config():
    """读取 config/launch.json; 缺失/损坏时返回空 dict 并告警。"""
    if _PROJECT_ROOT is None:
        print('[launch_common] 未找到 config/launch.json, 全部使用节点默认参数')
        return {}
    path = os.path.join(_PROJECT_ROOT, 'config', 'launch.json')
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        print(f'[launch_common] 读取 {path} 失败: {e}, 使用节点默认参数')
        return {}


def load_camera_config(key):
    """按 launch.json 里 camera_config_files 的角色读取相机 JSON。"""
    cfg = load_launch_config()
    files = cfg.get('camera_config_files', {})
    rel = files.get(key)
    if not rel or _PROJECT_ROOT is None:
        return {}
    path = os.path.join(_PROJECT_ROOT, rel)
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        print(f'[launch_common] 读取 {path} 失败: {e}, 该相机用 yaml 默认值')
        return {}


# ----------------------------------------------------------------------
# 参数覆盖小工具
# ----------------------------------------------------------------------

def truthy(text):
    """字符串转 bool(与各驱动的 sim:=true 语义一致)。"""
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


def sim_number(text, default):
    """解析时间窗参数: 空=默认, 'none'/负数=0(禁用该窗口)。"""
    text = text.strip()
    if text == '':
        return default
    if text.lower() == 'none':
        return 0.0
    try:
        return max(0.0, float(text))
    except ValueError:
        return default


# ----------------------------------------------------------------------
# 各节点构建(全部返回 launch_ros.actions.Node)
# ----------------------------------------------------------------------

def pkg_share(pkg):
    """返回功能包 share 目录(install 后的路径)。"""
    return get_package_share_directory(pkg)


def galaxy_node(args, log_level=''):
    """双大恒相机节点: yaml + 相机 JSON + launch 参数三层叠加。"""
    from launch_ros.actions import Node

    left = load_camera_config('left')
    right = load_camera_config('right')
    root_cfg = load_launch_config()
    sim_cfg = root_cfg.get('sim', {}).get('galaxy', {})

    overrides = {}
    # ---- 相机 JSON -> ROS 参数(设备 ID/别名/曝光等) ----
    for name, cam in (('left', left), ('right', right)):
        if not cam:
            continue
        p = f'cameras.{name}.'
        overrides[p + 'serial_number'] = str(cam.get('serial_number', ''))
        overrides[p + 'user_id'] = str(cam.get('user_id', ''))
        overrides[p + 'ip_address'] = str(cam.get('ip_address', ''))
        if cam.get('device_index') is not None:
            overrides[p + 'device_index'] = int(cam['device_index'])
        overrides[p + 'user_id_to_set'] = str(cam.get('user_id_to_set', ''))
        if cam.get('alias'):
            overrides[p + 'camera_name'] = str(cam['alias'])
        intr = cam.get('intrinsics', {})
        if intr.get('camera_info_url'):
            overrides[p + 'camera_info_url'] = str(intr['camera_info_url'])
        if cam.get('exposure_time_us') is not None:
            overrides[p + 'exposure_time'] = float(cam['exposure_time_us'])
        if cam.get('gain') is not None:
            overrides[p + 'gain'] = float(cam['gain'])
        if cam.get('frame_rate_hz') is not None:
            overrides[p + 'frame_rate'] = float(cam['frame_rate_hz'])
        if cam.get('packet_size') is not None:
            overrides[p + 'packet_size'] = int(cam['packet_size'])
        if cam.get('throughput_limit_bps') is not None:
            overrides[p + 'throughput_limit_bps'] = int(cam['throughput_limit_bps'])
        if cam.get('packet_delay') is not None:
            overrides[p + 'packet_delay'] = int(cam['packet_delay'])
        if cam.get('jpeg_quality') is not None:
            overrides[p + 'jpeg_quality'] = int(cam['jpeg_quality'])

    # ---- 全局 sim 时间窗(launch.json) ----
    overrides['sim_mode'] = root_cfg.get('sim_mode', True)
    absent = sim_number(args.get('absent', ''),
                        float(sim_cfg.get('initial_absent_sec', 2.0)))
    disc = sim_number(args.get('disc_at', ''),
                      float(sim_cfg.get('disconnect_at_sec', 0.0)))
    dur = sim_number(args.get('disc_dur', ''),
                     float(sim_cfg.get('disconnect_duration_sec', 4.0)))
    overrides['sim.initial_absent_sec'] = absent
    overrides['sim.disconnect_at_sec'] = disc
    overrides['sim.disconnect_duration_sec'] = dur

    # ---- 每相机独立覆盖(热插拔测试逐台控制) ----
    sim_all = root_cfg.get('sim', {})
    for name in ('left', 'right'):
        overrides[f'cameras.{name}.sim.initial_absent_sec'] = sim_number(
            args.get(f'{name}_absent', ''), absent)
        overrides[f'cameras.{name}.sim.disconnect_at_sec'] = sim_number(
            args.get(f'{name}_disc_at', ''), disc)
        overrides[f'cameras.{name}.sim.disconnect_duration_sec'] = sim_number(
            args.get(f'{name}_disc_dur', ''), dur)

    # ---- 每相机模拟轮播目录(launch.json, 左右不同场景; 工程根相对) ----
    if _PROJECT_ROOT:
        for name, key in (('left', 'sim_image_dir_left'),
                          ('right', 'sim_image_dir_right')):
            rel = sim_all.get(key, '')
            if rel:
                overrides[f'cameras.{name}.sim_image_dir'] = \
                    os.path.join(_PROJECT_ROOT, rel)

    # ---- 显式 launch 参数最终覆盖 ----
    if args.get('sim', '').strip() != '':
        overrides['sim_mode'] = truthy(args['sim'])

    return Node(
        package='galaxy_camera_dual',
        executable='galaxy_camera_dual_node',
        name='galaxy_camera_dual',
        output='screen',
        emulate_tty=True,
        **({'arguments': ['--ros-args', '--log-level', log_level]}
           if log_level in ('debug', 'info', 'warn', 'error') else {}),
        parameters=[os.path.join(
            pkg_share('galaxy_camera_dual'), 'config', 'camera_params.yaml'),
            overrides],
    )


def ir_node(args, log_level=''):
    """红外相机节点: yaml + infrared_camera.json + launch 参数。"""
    from launch_ros.actions import Node

    cam = load_camera_config('infrared')
    root_cfg = load_launch_config()
    sim_cfg = root_cfg.get('sim', {}).get('infrared', {})

    overrides = {'sim_mode': root_cfg.get('sim_mode', True)}
    if cam:
        if cam.get('device_index') is not None:
            overrides['device_index'] = int(cam['device_index'])
        if cam.get('device_path'):
            overrides['device_path'] = str(cam['device_path'])
        if cam.get('frame_rate_hz') is not None:
            overrides['frame_rate'] = float(cam['frame_rate_hz'])
        if cam.get('frame_id'):
            overrides['frame_id'] = str(cam['frame_id'])
    overrides['sim.initial_absent_sec'] = sim_number(
        args.get('ir_absent', ''), float(sim_cfg.get('initial_absent_sec', 2.0)))
    overrides['sim.disconnect_at_sec'] = sim_number(
        args.get('ir_disc_at', ''), float(sim_cfg.get('disconnect_at_sec', 0.0)))
    overrides['sim.disconnect_duration_sec'] = sim_number(
        args.get('ir_disc_dur', ''), float(sim_cfg.get('disconnect_duration_sec', 4.0)))
    if args.get('sim', '').strip() != '':
        overrides['sim_mode'] = truthy(args['sim'])

    return Node(
        package='ir_camera_driver',
        executable='ir_camera_driver_node',
        name='ir_camera_driver',
        output='screen',
        emulate_tty=True,
        **({'arguments': ['--ros-args', '--log-level', log_level]}
           if log_level in ('debug', 'info', 'warn', 'error') else {}),
        parameters=[os.path.join(
            pkg_share('ir_camera_driver'), 'config', 'ir_camera_params.yaml'),
            overrides],
    )


def sensor_node(args, log_level=''):
    """塔石温湿度/CO2 传感器节点: yaml + launch 参数。"""
    from launch_ros.actions import Node

    sim_cfg = load_launch_config().get('sim', {}).get('sensor', {})

    overrides = {'sim_mode': load_launch_config().get('sim_mode', True)}
    overrides['sim.initial_absent_sec'] = sim_number(
        args.get('sensor_absent', ''), float(sim_cfg.get('initial_absent_sec', 2.0)))
    overrides['sim.disconnect_at_sec'] = sim_number(
        args.get('sensor_disc_at', ''), float(sim_cfg.get('disconnect_at_sec', 0.0)))
    overrides['sim.disconnect_duration_sec'] = sim_number(
        args.get('sensor_disc_dur', ''), float(sim_cfg.get('disconnect_duration_sec', 4.0)))
    if args.get('sim', '').strip() != '':
        overrides['sim_mode'] = truthy(args['sim'])

    return Node(
        package='tas_sensor_driver',
        executable='tas_sensor_driver_node',
        name='tas_sensor_driver',
        output='screen',
        emulate_tty=True,
        **({'arguments': ['--ros-args', '--log-level', log_level]}
           if log_level in ('debug', 'info', 'warn', 'error') else {}),
        parameters=[os.path.join(
            pkg_share('tas_sensor_driver'), 'config', 'sensor_driver.yaml'),
            overrides],
    )


def core_node(args, log_level=''):
    """核心处理/同屏显示节点: yaml + launch 参数叠加。"""
    from launch_ros.actions import Node

    cfg = load_launch_config()
    overrides = {}
    if args.get('show_windows', '').strip() != '':
        overrides['show_img'] = truthy(args['show_windows'])
    elif isinstance(cfg.get('show_windows'), bool):
        overrides['show_img'] = cfg['show_windows']
    if args.get('save_dir', '').strip() != '':
        overrides['save_dir'] = args['save_dir'].strip()
    elif cfg.get('save_dir'):
        overrides['save_dir'] = cfg['save_dir']
    if args.get('yolo', '').strip() != '':
        overrides['yolo.enable_left'] = truthy(args['yolo'])
        overrides['yolo.enable_right'] = truthy(args['yolo'])
    elif isinstance(cfg.get('enable_yolo'), bool):
        overrides['yolo.enable_left'] = cfg['enable_yolo']
        overrides['yolo.enable_right'] = cfg['enable_yolo']
    if args.get('ir_seepage', '').strip() != '':
        overrides['ir.enable'] = truthy(args['ir_seepage'])
    elif isinstance(cfg.get('enable_ir_seepage'), bool):
        overrides['ir.enable'] = cfg['enable_ir_seepage']
    if isinstance(cfg.get('fullscreen'), bool):
        # 全屏展示: 窗口无边框占满整屏, 窗格高度按屏幕宽高比自动计算
        overrides['fullscreen'] = cfg['fullscreen']

    return Node(
        package='culvert_core',
        executable='core_node',
        name='core_node',
        output='screen',
        emulate_tty=True,
        **({'arguments': ['--ros-args', '--log-level', log_level]}
           if log_level in ('debug', 'info', 'warn', 'error') else {}),
        parameters=[os.path.join(
            pkg_share('culvert_core'), 'config', 'core_node_params.yaml'),
            overrides],
    )


def inspection_node(args, log_level=''):
    """巡检协议节点(uint8 0x01 -> 左右 t 后第 2 帧 -> 0x02)。"""
    from launch_ros.actions import Node

    cfg = load_launch_config()
    overrides = {}
    if args.get('in_trulyworking', '').strip() != '':
        overrides['in_trulyworking'] = truthy(args['in_trulyworking'])
    elif isinstance(cfg.get('in_trulyworking'), bool):
        overrides['in_trulyworking'] = cfg['in_trulyworking']
    if args.get('input_topic', '').strip() != '':
        overrides['input_topic'] = args['input_topic'].strip()
    elif cfg.get('input_topic'):
        overrides['input_topic'] = cfg['input_topic']
    if args.get('ack_topic', '').strip() != '':
        overrides['ack_topic'] = args['ack_topic'].strip()
    elif cfg.get('ack_topic'):
        overrides['ack_topic'] = cfg['ack_topic']
    if args.get('save_dir', '').strip() != '':
        overrides['save_dir'] = args['save_dir'].strip()
    elif cfg.get('save_dir'):
        overrides['save_dir'] = cfg['save_dir']
    if args.get('frame_timeout', '').strip() != '':
        overrides['frame_timeout_sec'] = float(args['frame_timeout'])
    elif cfg.get('frame_timeout_sec') is not None:
        overrides['frame_timeout_sec'] = float(cfg['frame_timeout_sec'])
    if cfg.get('target_frame_index') is not None:
        overrides['target_frame_index'] = int(cfg['target_frame_index'])

    return Node(
        package='culvert_inspection',
        executable='inspection_node',
        name='inspection_node',
        output='screen',
        emulate_tty=True,
        **({'arguments': ['--ros-args', '--log-level', log_level]}
           if log_level in ('debug', 'info', 'warn', 'error') else {}),
        parameters=[overrides],
    )


def standard_arguments():
    """all.launch.py 的通用声明参数(全部可空, 空=尊重 launch.json/yaml)。"""
    from launch.actions import DeclareLaunchArgument

    defs = [
        ('sim', 'true=全模拟, false=真机; 缺省=launch.json'),
        ('show_windows', '是否弹出同屏大面板; 缺省=launch.json'),
        ('in_trulyworking', '巡检模式开关; 缺省=launch.json'),
        ('sensor', '是否启动传感器驱动; 缺省=launch.json'),
        ('input_topic', '巡检指令输入话题(UInt8)'),
        ('ack_topic', '巡检确认输出话题(UInt8)'),
        ('save_dir', '巡检/调试图片保存目录'),
        ('frame_timeout', '巡检目标帧等待超时(秒)'),
        ('yolo', '左右 YOLO 推理开关'),
        ('ir_seepage', '红外渗水检测开关'),
        ('absent', '双大恒统一初始缺失时长(秒)'),
        ('disc_at', '双大恒统一中途断连时刻(秒), none=禁用'),
        ('disc_dur', '双大恒统一中途断连时长(秒)'),
        ('left_absent', '左大恒初始缺失时长(秒, 覆盖统一值)'),
        ('right_absent', '右大恒初始缺失时长(秒, 覆盖统一值)'),
        ('left_disc_at', '左大恒中途断连时刻(秒)'),
        ('left_disc_dur', '左大恒中途断连时长(秒)'),
        ('right_disc_at', '右大恒中途断连时刻(秒)'),
        ('right_disc_dur', '右大恒中途断连时长(秒)'),
        ('ir_absent', '红外初始缺失时长(秒)'),
        ('ir_disc_at', '红外中途断连时刻(秒)'),
        ('ir_disc_dur', '红外中途断连时长(秒)'),
        ('sensor_absent', '传感器初始缺失时长(秒)'),
        ('sensor_disc_at', '传感器中途断连时刻(秒)'),
        ('sensor_disc_dur', '传感器中途断连时长(秒)'),
        ('log_level', 'ROS 日志级别(debug/info/warn/error); 缺省=launch.json'),
    ]
    return [DeclareLaunchArgument(
        name=n, default_value='', description=d) for n, d in defs]


def log_level_arg(args):
    """取生效的 ROS 日志级别(供 --log-level), 未配置返回空。"""
    text = args.get('log_level', '').strip()
    if text:
        return text
    return str(load_launch_config().get('log_level', '')).strip()


def start_key_forward_if_tty():
    """启动终端按键转发(按 S -> /core_node/save_image; 非 tty 静默跳过)。"""
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from key_forward import start_key_forward
    start_key_forward()
