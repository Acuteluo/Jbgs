// Copyright (c) 2024, tas_sensor_driver contributors
//
// 文件: crc16.hpp
// 作用: Modbus-RTU 使用的 CRC16 校验(多项式 0xA001, 初值 0xFFFF, 低字节在前)。
//
// 已验证的测试向量(与《塔石传感器寄存器定义说明 V1.6》一致):
//   01 03 00 05 00 01  -> CRC = 0x0B94, 帧内字节顺序 94 0B
//   01 03 00 00 00 02  -> CRC = 0x0BC4, 帧内字节顺序 C4 0B
//   01 03 00 00 00 01  -> CRC = 0x0A84, 帧内字节顺序 84 0A
//   (注意: 使用说明书 4.4 节 CO2 示例的 CRC 写成了 C4 0B, 是文档笔误)
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tas_sensor_driver
{

/// 计算一段数据的 CRC16-Modbus 校验值。
/// data   : 待计算数据(不含 CRC 部分)
/// length : 数据长度(字节)
/// 返回   : CRC 值(高 8 位在前表示, 如 0x0B94)
uint16_t crc16Modbus(const uint8_t * data, std::size_t length);

/// vector 便捷重载。
uint16_t crc16Modbus(const std::vector<uint8_t> & data);

/// 把 CRC 追加到帧尾(低字节在前), 得到可直接发送的完整 Modbus-RTU 帧。
void appendCrc16(std::vector<uint8_t> & frame);

/// 校验整帧(帧尾已含 2 字节 CRC)是否正确。
/// frame : 完整帧(数据 + CRC)
/// 返回 : CRC 校验是否通过
bool verifyCrc16(const std::vector<uint8_t> & frame);

}  // namespace tas_sensor_driver
