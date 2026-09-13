# -*- coding: utf-8 -*-
#
# 文件: key_forward.py
# 作用: 终端按键转发工具(供 bringup 的 launch 文件调用)。
#
# 背景:
#   按 S 键保存当前一帧是 core_node 的功能(core_node 订阅话题
#   /core_node/save_image)。但 ros2 launch 启动的子进程 stdin 是
#   管道(不是终端), 子进程读不到键盘; 而 launch 进程本身的 stdin
#   就是用户终端。所以这里在 launch 进程内起一个后台线程读终端,
#   检测到 S/s 键后, 通过 rclpy 发布一条 std_msgs/Empty 到
#   /core_node/save_image, core_node 收到即保存一帧 JPEG。
#
# 用法(在 launch 文件 generate_launch_description() 里):
#   sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
#   from key_forward import start_key_forward
#   start_key_forward()   # 返回前会启动后台线程, 不阻塞
#
# 注意:
#   - 仅当 launch 进程的 stdin 是终端时才生效; 否则静默跳过;
#   - 退出时自动恢复终端原始设置(raw 模式 -> 正常模式)。

import atexit
import os
import select
import sys
import termios
import threading

import rclpy
from std_msgs.msg import Empty


def _key_reader(save_topic: str) -> None:
    """后台线程: 读终端 stdin, 检测到 S/s 就发布保存请求。"""
    fd = sys.stdin.fileno()

    # 只有 stdin 是终端才监听(ros2 launch 从终端运行时成立)
    if not os.isatty(fd):
        return

    # 保存原终端设置, 供退出时恢复
    old_attr = termios.tcgetattr(fd)

    def restore():
        try:
            termios.tcsetattr(fd, termios.TCSANOW, old_attr)
        except Exception:
            pass

    atexit.register(restore)   # launch 进程退出(SIGINT)时恢复终端

    # 原始模式: 去掉行缓冲与回显, 按键立刻可见;
    # 保留 ISIG, 这样 Ctrl+C 仍然产生 SIGINT, launch 正常退出。
    new_attr = termios.tcgetattr(fd)
    new_attr[3] &= ~(termios.ICANON | termios.ECHO)   # lflag
    new_attr[6][termios.VMIN] = 0                     # cc
    new_attr[6][termios.VTIME] = 2                    # cc(200ms 超时)
    termios.tcsetattr(fd, termios.TCSANOW, new_attr)

    # rclpy 在 launch 进程内初始化并发布(已验证可用)
    rclpy.init()
    node = rclpy.create_node("key_forward")
    pub = node.create_publisher(Empty, save_topic, 10)

    print("key_forward: 已启用, 按 S 键保存当前一帧 -> %s" % save_topic,
          flush=True)

    try:
        while True:
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                continue
            try:
                ch = os.read(fd, 1)
            except OSError:
                break
            if ch in (b"s", b"S"):
                pub.publish(Empty())
                print("key_forward: S 键, 已请求保存当前一帧", flush=True)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        restore()


def start_key_forward(save_topic: str = "/core_node/save_image") -> None:
    """启动按键转发后台线程(立即返回, 不阻塞 launch)。"""
    threading.Thread(
        target=_key_reader,
        args=(save_topic,),
        daemon=True,
    ).start()
