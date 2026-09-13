// Copyright (c) 2024, tas_sensor_driver contributors
//
// 文件: modbus.hpp
// 作用: Modbus-RTU 帧的打包与解析(仿旧项目 packet.hpp 的角色)。
//
// 协议要点(见《塔石传感器寄存器定义说明 V1.6》):
//  - 问询帧: [地址][功能码][寄存器起始高][低][长度高][低][CRC低][CRC高]
//  - 读应答: [地址][功能码][字节数 N][数据 N 字节][CRC低][CRC高]
//  - 功能码 0x03/0x04 读, 0x06 写单, 0x10 写多
#pragma once

#include <cstdint>
#include <vector>

namespace tas_sensor_driver
{

/// 读寄存器功能码(0x03)。
constexpr uint8_t kFuncRead = 0x03;

/// 传感器寄存器地址(本产品 = 温湿度 + CO2):
///  - 湿度 0x0000, 1 个寄存器, 值/10 = %RH
///  - 温度 0x0001, 1 个寄存器, 16 位补码, 值/10 = ℃(0xFF9B = -10.1℃)
///  - CO2  0x0005, 1 个寄存器, 原值即 ppm
constexpr uint16_t kRegHumidity = 0x0000;
constexpr uint16_t kRegTemperature = 0x0001;
constexpr uint16_t kRegCo2 = 0x0005;

/// 构造"读寄存器"问询帧(自动附加 CRC)。
/// addr  : 从机 Modbus 地址(出厂默认 1)
/// reg   : 起始寄存器地址
/// count : 连续读取的寄存器个数
std::vector<uint8_t> buildReadRequest(
    uint8_t addr, uint16_t reg, uint16_t count);

/// 解析"读寄存器"应答帧(帧尾已含 CRC), 提取数据寄存器值。
/// frame  : 完整应答帧
/// addr   : 期望的从机地址(与问询地址比对, 防止串扰)
/// values : 输出, 按顺序存放各寄存器原始值(16 位)
/// 返回   : 地址/功能码/长度/CRC 全部正确才为 true
bool parseReadResponse(
    const std::vector<uint8_t> & frame, uint8_t addr,
    std::vector<uint16_t> * values);

}  // namespace tas_sensor_driver
