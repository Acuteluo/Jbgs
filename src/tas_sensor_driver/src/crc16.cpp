// Copyright (c) 2024, tas_sensor_driver contributors
//
// 文件: crc16.cpp
// 作用: CRC16-Modbus 的实现(逐位查表法之外的经典逐位算法)。

#include "tas_sensor_driver/crc16.hpp"

namespace tas_sensor_driver
{

uint16_t crc16Modbus(const uint8_t * data, std::size_t length)
{
    // 初值 0xFFFF(Modbus 标准)
    uint16_t crc = 0xFFFF;

    for (std::size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        // 多项式 0xA001(0x8005 的位反转), 右移 8 次
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t crc16Modbus(const std::vector<uint8_t> & data)
{
    return crc16Modbus(data.data(), data.size());
}

void appendCrc16(std::vector<uint8_t> & frame)
{
    const uint16_t crc = crc16Modbus(frame);
    // Modbus-RTU 规定 CRC 低字节在前
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc >> 8));
}

bool verifyCrc16(const std::vector<uint8_t> & frame)
{
    if (frame.size() < 3)
    {
        return false;
    }
    // 帧尾 2 字节即发送方附加的 CRC(低字节在前)
    const size_t data_len = frame.size() - 2;
    const uint16_t received =
        static_cast<uint16_t>(frame[data_len]) |
        (static_cast<uint16_t>(frame[data_len + 1]) << 8);
    return received == crc16Modbus(frame.data(), data_len);
}

}  // namespace tas_sensor_driver
