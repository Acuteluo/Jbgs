// Copyright (c) 2024, tas_sensor_driver contributors
//
// 文件: tas_sensor_driver.hpp
// 作用: 塔石 TAS-WSCO2 温湿度二氧化碳传感器驱动节点声明。
//
// 传输层说明(重要):
//  本版不使用 serial_driver 库, 而是直接 POSIX termios 读写串口:
//   - 串口设 VMIN=0/VTIME=2, 每次 read() 最多阻塞 200ms, 总超时由
//     轮询线程用截止时间控制 —— 阻塞读可控、可超时;
//   - 问询超时后 tcflush(TCIFLUSH) 清空输入缓冲 —— 迟到的/半截的
//     旧应答被彻底清掉, 不会"串代"到下一问询(此前用 serial_driver
//     异步接口时, 重开串口会取消在途操作并刷屏 Operation aborted,
//     且残留字节导致数据震荡, 是现场反复出问题的根源);
//   - 设备被拔: write/read 返回 EIO -> 关 fd, 每轮重试 open(), 插回即恢复;
//   - 单线程同步收发, 无锁、无异步回调、退出无挂死。
//
// 线程模型:
//  - poll_thread_ 轮询线程(唯一自建线程): 每 1s 发两帧问询
//      (温湿度 0x0000 起 2 寄存器, CO2 0x0005 起 1 寄存器),
//      同步等应答(默认 500ms 超时), 两次问询间隔 ≥200ms;
//  - 服务回调(执行器线程): 置 force_query_ 插队, 等新一轮数据后返回。
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_interfaces/msg/env_data.hpp>
#include <sensor_interfaces/srv/query_env.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tas_sensor_driver
{

class TasSensorDriver : public rclcpp::Node
{
public:
    explicit TasSensorDriver(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~TasSensorDriver() override;

private:
    // ==================== 串口(POSIX termios) ====================

    /// 打开串口并配置 9600 8N1 + VMIN=0/VTIME=2(200ms 读超时)。
    /// 返回是否成功(失败时 fd_ 保持 -1)。
    bool openSerial();

    /// 关闭串口(fd_ 置 -1)。幂等。
    void closeSerial();

    /// 发送完整一帧(处理部分写与 EINTR)。返回是否成功。
    bool sendFrame(const std::vector<uint8_t> & frame);

    /// 接收一帧应答(带总超时)。
    /// 返回: true=组满一帧且 CRC 通过; false=超时/读错误/CRC 失败。
    /// 超时返回时内部已 tcflush 清空输入缓冲, 防止残帧串代。
    bool receiveFrame(std::vector<uint8_t> * frame, int timeout_ms);

    // ==================== 轮询 ====================

    /// 轮询主循环: 发问询 -> 收应答 -> 解析 -> 发布 EnvData。
    void pollLoop();

    /// 一次"读寄存器"问询(同步收发)。
    bool queryRegisters(
        uint16_t reg, uint16_t count, std::vector<uint16_t> * values);

    /// 模拟模式: 一轮"虚拟问询"。
    /// 在位时按时间生成缓慢变化的温湿度/CO2(正弦 + 噪声), 返回 true;
    /// 离线时间窗内返回 false(模拟传感器被拔), 与真机失败路径同构,
    /// 发布端复用同一套 valid 标志/发布逻辑 —— 保证热插拔测试代表性。
    bool queryRegistersSim(
        uint16_t reg, uint16_t count, std::vector<uint16_t> * values);

    /// 模拟模式: 当前传感器是否在位(不在离线时间窗内)。
    bool simDevicePresent() const;

    // ==================== 初始化 ====================

    /// 声明并读取参数(串口参数 + Modbus 地址 + 轮询/超时配置)。
    void getParams();

    // ==================== 发布与服务 ====================

    /// 把最新数据组装成 EnvData 发布到 /sensor/env。
    void publishEnv();

    /// 查询服务回调: 置 force_query_ 插队, 等待新一轮数据后返回。
    void queryEnvCallback(
        const sensor_interfaces::srv::QueryEnv::Request::SharedPtr request,
        const sensor_interfaces::srv::QueryEnv::Response::SharedPtr response);

    /// 动态参数回调(轮询间隔/超时/日志开关可在线改)。
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & params);

    // ==================== 串口状态 ====================

    int fd_ = -1;                     ///< 串口文件描述符(-1 = 未打开)

    // ==================== 线程与同步 ====================

    std::atomic<bool> running_{false};   ///< 节点运行标志(原子)
    std::thread poll_thread_;            ///< 轮询线程(唯一自建线程)

    std::mutex reply_mutex_;             ///< 最新数据的互斥(轮询线程写/服务读)
    std::condition_variable query_cv_;   ///< 服务等待新一轮数据
    bool force_query_ = false;           ///< 服务请求的插队标志

    std::chrono::steady_clock::time_point last_query_time_{};  ///< 上次发帧时刻(控制 ≥200ms 间隔)

    // ==================== 参数(来自 config/sensor_driver.yaml) ====================

    std::string device_name_;    ///< 串口设备, 默认 /dev/ttyUSB0
    uint32_t baud_rate_ = 9600;  ///< 波特率, 出厂 9600
    std::string parity_ = "none";     ///< none / odd / even
    std::string stop_bits_ = "1";     ///< 1 / 2
    std::string flow_control_ = "none";  ///< none / hardware
    int modbus_addr_ = 1;        ///< 从机 Modbus 地址, 出厂 1
    int poll_interval_ms_ = 1000;   ///< 轮询周期(传感器 2s 更新, 1Hz 合理)
    int query_timeout_ms_ = 500;    ///< 单次问询等应答超时(≥200ms)
    int inter_query_gap_ms_ = 200;  ///< 两次问询最小间隔(文档要求 ≥200ms)
    bool show_logger_ = true;       ///< 是否打印读数日志

    // ==================== 模拟模式参数(sim_mode=true 时生效) ====================

    bool sim_mode_ = false;             ///< true = 无实体传感器, 用模拟数据
    double sim_initial_absent_sec_ = 5.0;   ///< 启动后模拟"未接入"时长(秒)
    double sim_disconnect_at_sec_ = 40.0;   ///< 首次"中途拔掉"时刻(秒); <=0 关闭
    double sim_disconnect_duration_sec_ = 10.0;  ///< 每次中途离线时长(秒)
    double sim_disconnect_repeat_period_sec_ = 0.0;  ///< >0 = 周期重复断连
    float sim_base_temperature_ = 25.0f;    ///< 模拟温度基准(℃)
    float sim_base_humidity_ = 60.0f;       ///< 模拟湿度基准(%RH)
    float sim_base_co2_ = 650.0f;           ///< 模拟 CO2 基准(ppm)
    std::chrono::steady_clock::time_point sim_start_time_{};  ///< 模拟时间窗基准

    // ==================== 最新数据(由 reply_mutex_ 保护) ====================

    bool temp_hum_valid_ = false;   ///< 温湿度最近一次是否读到
    bool co2_valid_ = false;        ///< CO2 最近一次是否读到
    bool prev_cycle_failed_ = false;  ///< 上一轮询周期是否完全失败(恢复日志用)
    float temperature_ = 0.0f;      ///< ℃
    float humidity_ = 0.0f;         ///< %RH
    float co2_ = 0.0f;              ///< ppm
    rclcpp::Time last_update_;      ///< 最近一次成功应答时刻
    uint64_t cycle_count_ = 0;      ///< 完成的轮询周期计数

    // ==================== ROS2 通信对象 ====================

    rclcpp::Publisher<sensor_interfaces::msg::EnvData>::SharedPtr env_pub_;
    rclcpp::Service<sensor_interfaces::srv::QueryEnv>::SharedPtr query_srv_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace tas_sensor_driver
