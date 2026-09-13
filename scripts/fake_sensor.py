#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器模拟器 —— 传感器断电/未接时联调下游。

用法:
  python3 scripts/fake_sensor.py [--rate 1.0]

发布话题(与真实 tas_sensor_driver 完全一致):
  /sensor/env  sensor_interfaces/msg/EnvData
    temp_hum_valid=True, co2_valid=True
    温度 24~26℃ 缓慢正弦漂移, 湿度 50~60%, CO2 600~900ppm 呼吸式波动
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_interfaces.msg import EnvData


class FakeSensor(Node):
    def __init__(self, rate):
        super().__init__("fake_sensor")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=3)
        self.pub = self.create_publisher(EnvData, "/sensor/env", qos)
        self.rate = rate
        self.n = 0
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"模拟传感器启动: {rate}Hz")

    def tick(self):
        self.n += 1
        t = self.n / self.rate
        m = EnvData()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "env_sensor"
        m.temp_hum_valid = True
        m.co2_valid = True
        m.temperature = 25.0 + 1.2 * math.sin(t / 30.0)      # 24~26℃
        m.humidity = 55.0 + 4.0 * math.sin(t / 45.0 + 1.0)   # 51~59%
        m.co2 = 750.0 + 120.0 * math.sin(t / 60.0)           # 630~870ppm
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=1.0)
    args = ap.parse_args()
    rclpy.init()
    node = FakeSensor(args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
