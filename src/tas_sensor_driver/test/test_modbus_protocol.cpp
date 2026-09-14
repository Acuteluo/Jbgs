// Copyright (c) 2026, tas_sensor_driver contributors
//
// 文件: test_modbus_protocol.cpp
// 作用: 按塔石《寄存器定义说明 V1.6》示例校验 Modbus-RTU 报文。

#include "tas_sensor_driver/crc16.hpp"
#include "tas_sensor_driver/modbus.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace tas_sensor_driver
{
namespace
{

TEST(ModbusProtocol, FactoryReadRequestsMatchManual)
{
    EXPECT_EQ(
        buildReadRequest(0x01, kRegHumidity, 2),
        (std::vector<uint8_t>{0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B}));
    EXPECT_EQ(
        buildReadRequest(0x01, kRegCo2, 1),
        (std::vector<uint8_t>{0x01, 0x03, 0x00, 0x05, 0x00, 0x01, 0x94, 0x0B}));
}

TEST(ModbusProtocol, FactoryResponsesDecodeValues)
{
    std::vector<uint16_t> values;
    // V1.6: 0x02B5 -> 69.3 %RH.
    EXPECT_TRUE(parseReadResponse(
        {0x01, 0x03, 0x02, 0x02, 0xB5, 0x78, 0x93}, 0x01, &values));
    ASSERT_EQ(values.size(), 1U);
    EXPECT_EQ(values[0], 0x02B5);

    // V1.6: 0x0108 -> 26.4 C.
    EXPECT_TRUE(parseReadResponse(
        {0x01, 0x03, 0x02, 0x01, 0x08, 0xB8, 0x12}, 0x01, &values));
    ASSERT_EQ(values.size(), 1U);
    EXPECT_EQ(values[0], 0x0108);

    // V1.6: 0x0279 -> 633 ppm.
    EXPECT_TRUE(parseReadResponse(
        {0x01, 0x03, 0x02, 0x02, 0x79, 0x78, 0xC6}, 0x01, &values));
    ASSERT_EQ(values.size(), 1U);
    EXPECT_EQ(values[0], 0x0279);
}

TEST(ModbusProtocol, RejectsForeignOrMalformedResponse)
{
    std::vector<uint16_t> values;
    EXPECT_FALSE(parseReadResponse(
        {0x02, 0x03, 0x02, 0x02, 0xB5, 0x3C, 0x93}, 0x01, &values));

    // CRC is valid, but a register response must have an even number of data bytes.
    std::vector<uint8_t> odd_data{0x01, 0x03, 0x01, 0x7F};
    appendCrc16(odd_data);
    EXPECT_TRUE(verifyCrc16(odd_data));
    EXPECT_FALSE(parseReadResponse(odd_data, 0x01, &values));
}

}  // namespace
}  // namespace tas_sensor_driver
