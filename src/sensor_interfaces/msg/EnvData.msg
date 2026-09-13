# 塔石 TAS-WSCO2 温湿度二氧化碳传感器的环境数据
# 由 tas_sensor_driver 节点以 1Hz 轮询后发布到 /sensor/env 话题

std_msgs/Header header    # 时间戳 = 最近一次成功应答的接收时刻

bool temp_hum_valid       # 温湿度本次轮询是否读取成功
bool co2_valid            # CO2 本次轮询是否读取成功

float32 temperature       # 温度(℃), 寄存器 0x0001, 16位补码 /10
float32 humidity          # 湿度(%RH), 寄存器 0x0000, /10
float32 co2               # 二氧化碳浓度(ppm), 寄存器 0x0005, 原值
