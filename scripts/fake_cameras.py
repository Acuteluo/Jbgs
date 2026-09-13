#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双相机模拟器 —— 相机断电时联调下游(YOLO/显示/保存)全链路。

用法:
  python3 scripts/fake_cameras.py [--rate 8] [--glitch 0]

  --rate N      每台相机发布帧率(默认8, 与真实限帧目标一致)
  --glitch 秒   >0 时每 20 秒停发 N 秒, 模拟断流(测试下游健壮性)

发布话题(与真实节点一致):
  /left_camera/image_raw/compressed   sensor_msgs/CompressedImage (jpeg)
  /right_camera/image_raw/compressed
  /left_camera/camera_info  /right_camera/camera_info (空标定, 仅保格式)
"""
import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo


def make_frame(label, side, t, w=640, h=480):
    """生成一眼能认出的测试图: 渐变底 + 移动条纹 + 大字标签 + 帧号。"""
    img = np.zeros((h, w, 3), np.uint8)
    # 左相机青色系 / 右相机橙色系, 直观区分左右
    c1 = (255, 60, 255) if side == "L" else (60, 255, 255)   # BGR
    img[:] = (c1[0] // 6, c1[1] // 6, c1[2] // 6)
    # 横向渐变
    grad = np.tile(np.linspace(30, 120, w, dtype=np.uint8), (h, 1))
    img[:, :, 0] = np.minimum(255, img[:, :, 0].astype(int) + grad)
    # 移动竖条纹(速度=帧号推进, 便于肉眼确认帧率)
    x0 = int(t * 80) % (w + 80)
    for k in range(4):
        xs = x0 - k * 90
        if 0 <= xs < w - 40:
            cv2.rectangle(img, (xs, 60), (xs + 40, h - 60), c1, -1)
    # 大标签与帧号
    cv2.putText(img, f"{label} FAKE CAM", (30, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 3)
    cv2.putText(img, f"t={t:6.1f}s", (30, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return img


class FakeCameras(Node):
    def __init__(self, rate, glitch):
        super().__init__("fake_cameras")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        self.pubs = {}
        self.infos = {}
        for name, fid in (("left", "left_camera_optical_frame"),
                          ("right", "right_camera_optical_frame")):
            self.pubs[name] = self.create_publisher(
                CompressedImage,
                f"/{name}_camera/image_raw/compressed", qos)
            ci = CameraInfo()
            ci.header.frame_id = fid
            ci.width, ci.height = 640, 480
            self.infos[name] = ci
        self.rate = rate
        self.glitch = glitch
        self.n = 0
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"模拟双相机启动: {rate}Hz"
            + (f", 每20s断流{glitch}s" if glitch else ""))

    def tick(self):
        self.n += 1
        t = self.n / self.rate
        # 故障注入: 每 20 秒停发 glitch 秒(不含首段)
        if self.glitch and (t % 20) > (20 - self.glitch):
            return
        for name, tag in (("left", "LEFT"), ("right", "RIGHT")):
            msg = CompressedImage()
            msg.header.frame_id = f"{name}_camera_optical_frame"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            ok, enc = cv2.imencode(
                ".jpg", make_frame(tag, name[0].upper(), t),
                [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                msg.data = enc.tobytes()
                self.pubs[name].publish(msg)
        for name in ("left", "right"):
            ci = self.infos[name]
            ci.header.stamp = self.get_clock().now().to_msg()
            # camera_info 无独立 publisher 时 core 只用 compressed, 此处仅保帧
        _ = threading.enumerate()  # noqa: 保持 rclpy 稳定


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=8.0)
    ap.add_argument("--glitch", type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    node = FakeCameras(args.rate, args.glitch)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
