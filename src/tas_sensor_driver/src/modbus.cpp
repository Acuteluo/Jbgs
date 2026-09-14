// Copyright (c) 2024, tas_sensor_driver contributors
//
// 文件: modbus.cpp
// 作用: Modbus-RTU 帧的打包与解析实现。

#include "tas_sensor_driver/modbus.hpp"

#include "tas_sensor_driver/crc16.hpp"

namespace tas_sensor_driver
{

std::vector<uint8_t> buildReadRequest(
    uint8_t addr, uint16_t reg, uint16_t count)
{
    // 问询帧: [地址][功能码03][寄存器高][低][数量高][低][CRC低][CRC高]
    std::vector<uint8_t> frame;
    frame.push_back(addr);
    frame.push_back(kFuncRead);
    frame.push_back(static_cast<uint8_t>(reg >> 8));
    frame.push_back(static_cast<uint8_t>(reg & 0xFF));
    frame.push_back(static_cast<uint8_t>(count >> 8));
    frame.push_back(static_cast<uint8_t>(count & 0xFF));
    appendCrc16(frame);
    return frame;
}

bool parseReadResponse(
    const std::vector<uint8_t> & frame, uint8_t addr,
    std::vector<uint16_t> * values)
{
    // 读应答: [地址][功能码03][字节数N][数据N字节][CRC低][CRC高]
    if (values == nullptr || frame.size() < 5)
    {
        return false;
    }
    // 地址必须与问询一致(总线上只有一台设备, 这里是防串扰兜底)
    if (frame[0] != addr)
    {
        return false;
    }
    if (frame[1] != kFuncRead)
    {
        return false;
    }
    if (!verifyCrc16(frame))
    {
        return false;
    }

    const size_t byte_count = frame[2];
    // Modbus 寄存器固定 16 位；奇数字节数不是合法的读寄存器应答，
    // 不能悄悄丢掉最后一个字节后继续发布错误数值。
    if (byte_count == 0 || (byte_count % 2) != 0 ||
        frame.size() != 5 + byte_count)
    {
        return false;
    }

    values->clear();
    for (size_t i = 0; i + 1 < byte_count; i += 2)
    {
        const uint16_t value = static_cast<uint16_t>(
            (static_cast<uint16_t>(frame[3 + i]) << 8) | frame[4 + i]);
        values->push_back(value);
    }
    return !values->empty();
}

}  // namespace tas_sensor_driver
