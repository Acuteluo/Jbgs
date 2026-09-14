// Copyright (c) 2024, tas_sensor_driver contributors
//
// 文件: tas_sensor_driver.cpp
// 作用: 塔石 TAS-WSCO2 温湿度二氧化碳传感器驱动实现(POSIX termios 版)。
//
// 硬件与协议前提(见产品资料):
//  - RS485(Modbus-RTU), 出厂 9600 8N1, 地址 0x01;
//  - 寄存器: 湿度 0x0000(/10), 温度 0x0001(16位补码/10), CO2 0x0005(ppm);
//  - 两次问询间隔与等待应答必须 ≥200ms; 传感器数值 2s 更新一次。
//
// 为什么不用 serial_driver 库的异步接口(踩坑记录):
//  - 它的 async_receive 在串口被关闭时会刷屏 "Operation aborted"(一次
//    上百条), 且重开后输入缓冲里的残帧会"串代"到下一问询, 导致数据
//    时而有效时而无效的恶性震荡;
//  - 改用 POSIX termios 后: 阻塞读有 200ms 超时、超时后 tcflush 清
//    输入缓冲、单线程同步收发 —— 拔插行为完全确定。

#include "tas_sensor_driver/tas_sensor_driver.hpp"

#include "tas_sensor_driver/crc16.hpp"
#include "tas_sensor_driver/modbus.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <chrono>
#include <cstring>
#include <string>
#include <typeinfo>   // typeid: 异常日志里打印异常类型

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace tas_sensor_driver
{

namespace
{
/// 波特率数值 -> termios 常量(支持产品资料列出的全部档位)。
speed_t baudToTermios(uint32_t baud)
{
    switch (baud)
    {
        case 1200:   return B1200;
        case 2400:   return B2400;
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
    }
}
}  // namespace

// ============================== 构造 ==============================

TasSensorDriver::TasSensorDriver(const rclcpp::NodeOptions & options)
: Node("tas_sensor_driver", options)
{
    RCLCPP_INFO(get_logger(), "TasSensorDriver 启动");

    // 1. 读取参数(串口/地址/轮询配置)
    getParams();

    // 2. ROS2 通信对象: 实时话题 + 查询服务
    env_pub_ = create_publisher<sensor_interfaces::msg::EnvData>(
        "/sensor/env", rclcpp::SensorDataQoS());
    query_srv_ = create_service<sensor_interfaces::srv::QueryEnv>(
        "/sensor/query",
        [this](
            const sensor_interfaces::srv::QueryEnv::Request::SharedPtr req,
            const sensor_interfaces::srv::QueryEnv::Response::SharedPtr res)
        {
            queryEnvCallback(req, res);
        });

    // 3. 动态参数回调(轮询间隔/超时/日志可在线改)
    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params)
        {
            return onParameterChange(params);
        });

    // 4. 尝试打开串口(失败不退出, 轮询线程每轮都会重试)。
    //    模拟模式跳过串口: 不触碰 /dev/ttyUSB0, 数据由 queryRegistersSim 生成。
    running_ = true;
    if (sim_mode_)
    {
        RCLCPP_INFO(get_logger(), "模拟模式: 跳过串口打开");
    }
    else if (!openSerial())
    {
        RCLCPP_WARN(
            get_logger(),
            "打开串口失败(设备=%s): %s —— 将每轮重试, 请检查接线/权限"
            "(sudo usermod -aG dialout $USER)",
            device_name_.c_str(), std::strerror(errno));
    }

    // 5. 启动轮询线程
    poll_thread_ = std::thread(&TasSensorDriver::pollLoop, this);

    RCLCPP_INFO(get_logger(), "TasSensorDriver 初始化完成");
}

// ============================== 析构 ==============================
// 顺序: 置停止标志 -> 关串口(唤醒阻塞读) -> join 轮询线程。
// 读带 200ms 超时, 因此 join 最多等 200ms, 不会挂死。
TasSensorDriver::~TasSensorDriver()
{
    RCLCPP_INFO(get_logger(), "TasSensorDriver 关闭中...");
    running_ = false;

    closeSerial();   // 让轮询线程里的阻塞 read 立刻以 EBADF 返回

    if (poll_thread_.joinable())
    {
        poll_thread_.join();
    }

    RCLCPP_INFO(get_logger(), "TasSensorDriver 已退出");
}

// ============================== 参数 ==============================

void TasSensorDriver::getParams()
{
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    baud_rate_ = static_cast<uint32_t>(
        declare_parameter<int>("baud_rate", 9600));
    parity_ = declare_parameter<std::string>("parity", "none");
    stop_bits_ = declare_parameter<std::string>("stop_bits", "1");
    flow_control_ = declare_parameter<std::string>("flow_control", "none");

    modbus_addr_ = declare_parameter<int>("modbus_addr", 1);
    poll_interval_ms_ = declare_parameter<int>("poll_interval_ms", 1000);
    query_timeout_ms_ = declare_parameter<int>("query_timeout_ms", 500);
    inter_query_gap_ms_ = declare_parameter<int>("inter_query_gap_ms", 200);
    show_logger_ = declare_parameter<bool>("show_logger", true);

    // ---- 模拟模式参数(sim_mode=true 时跳过串口, 数据/离线由模拟生成,
    //      离线时间窗语义与 galaxy/ir 相机的模拟设备同构) ----
    sim_mode_ = declare_parameter<bool>("sim_mode", false);
    sim_initial_absent_sec_ = declare_parameter<double>("sim.initial_absent_sec", 5.0);
    sim_disconnect_at_sec_ = declare_parameter<double>("sim.disconnect_at_sec", 40.0);
    sim_disconnect_duration_sec_ =
        declare_parameter<double>("sim.disconnect_duration_sec", 10.0);
    sim_disconnect_repeat_period_sec_ =
        declare_parameter<double>("sim.disconnect_repeat_period_sec", 0.0);
    sim_base_temperature_ =
        static_cast<float>(declare_parameter<double>("sim.base_temperature", 25.0));
    sim_base_humidity_ =
        static_cast<float>(declare_parameter<double>("sim.base_humidity", 60.0));
    sim_base_co2_ =
        static_cast<float>(declare_parameter<double>("sim.base_co2", 650.0));
    sim_start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
        get_logger(),
        "参数: device=%s baud=%u parity=%s stop=%s flow=%s addr=%d "
        "poll=%dms timeout=%dms gap=%dms",
        device_name_.c_str(), baud_rate_, parity_.c_str(), stop_bits_.c_str(),
        flow_control_.c_str(), modbus_addr_,
        poll_interval_ms_.load(), query_timeout_ms_.load(), inter_query_gap_ms_.load());
    if (sim_mode_)
    {
        RCLCPP_INFO(
            get_logger(),
            "★ 模拟模式: 无实体传感器, 跳过串口。"
            "初始离线 %.1fs, 中途断连 %.1fs 起 / %.1fs 长 / 重复周期 %.1fs",
            sim_initial_absent_sec_, sim_disconnect_at_sec_,
            sim_disconnect_duration_sec_, sim_disconnect_repeat_period_sec_);
    }
}

// ============================== 串口(POSIX termios) ==============================

bool TasSensorDriver::openSerial()
{
    closeSerial();

    // 注意: 必须以 O_NONBLOCK 打开! 否则某些状态下(刚插拔过的 USB 转串口)
    // open() 会阻塞在 tty_port_block_til_ready 等载波, 永远不返回。
    fd_ = ::open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        fd_ = -1;
        return false;
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    cfmakeraw(&tio);   // 原始模式(8 数据位, 无校验, 1 停止位)
    // cfmakeraw 不保证置 CLOCAL/CREAD；尤其由 memset 初始化 termios 时，
    // 未显式置 CREAD 会让 USB-RS485 出现“端口已打开但始终收不到应答”。
    tio.c_cflag |= CLOCAL | CREAD;

    const speed_t speed = baudToTermios(baud_rate_);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    // 校验位
    if (parity_ == "odd")
    {
        tio.c_cflag |= PARENB | PARODD;
    }
    else if (parity_ == "even")
    {
        tio.c_cflag |= PARENB;
    }
    // 停止位
    if (stop_bits_ == "2")
    {
        tio.c_cflag |= CSTOPB;
    }
    // 硬件流控
    if (flow_control_ == "hardware")
    {
        tio.c_cflag |= CRTSCTS;
    }

    // 读超时: VMIN=0, VTIME=2 -> 每次 read() 最多等 200ms,
    // 无数据时返回 0; 总超时由上层截止时间控制。
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 2;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
        closeSerial();
        return false;
    }

    // 清除 O_NONBLOCK: 之后 read() 按 VMIN/VTIME 阻塞(可控超时)
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags >= 0)
    {
        ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void TasSensorDriver::closeSerial()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool TasSensorDriver::sendFrame(const std::vector<uint8_t> & frame)
{
    size_t offset = 0;
    while (offset < frame.size())
    {
        const ssize_t n = ::write(fd_, frame.data() + offset, frame.size() - offset);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            // EIO 等: 设备被拔/出错
            return false;
        }
        // 阻塞 fd 上 write 返回 0 不代表已写入任何字节；若继续循环会空转。
        if (n == 0)
        {
            errno = EIO;
            return false;
        }
        offset += static_cast<size_t>(n);
    }
    return true;
}

bool TasSensorDriver::receiveFrame(
    std::vector<uint8_t> * frame, int timeout_ms, uint8_t expected_addr)
{
    frame->clear();
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    size_t total_needed = 0;   // 0 = 还在等帧头

    while (true)
    {
        // 总超时检查
        const auto remain = deadline - std::chrono::steady_clock::now();
        if (remain <= std::chrono::milliseconds(0))
        {
            // 清掉可能还在路上的迟到/半截帧, 防止串到下一问询
            tcflush(fd_, TCIFLUSH);
            errno = ETIMEDOUT;
            return false;
        }

        uint8_t buffer[64];
        const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            // EIO/EBADF: 设备被拔/串口被关
            return false;
        }
        if (n == 0)
        {
            // VTIME 到期无数据, 继续等(总超时未到)
            continue;
        }

        frame->insert(frame->end(), buffer, buffer + n);

        // ---- 帧头阶段: 需要 地址+功能码 2 字节 ----
        if (total_needed == 0)
        {
            if (frame->size() < 2)
            {
                continue;
            }
            // RS485 总线上可能有其他地址的应答。丢弃其帧头并继续同步，
            // 不能把它当作本机问询的超时或错误应答。
            if ((*frame)[0] != expected_addr)
            {
                frame->erase(frame->begin());
                continue;
            }
            const uint8_t func = (*frame)[1];
            if (func == kFuncRead || func == 0x04)
            {
                if (frame->size() < 3)
                {
                    continue;   // 还差"字节数"域
                }
                total_needed = 3 + static_cast<size_t>((*frame)[2]) + 2;
            }
            else if (func == (kFuncRead | 0x80) || func == 0x84)
            {
                total_needed = 5;   // 异常帧: 头2 + 异常码1 + CRC2
            }
            else
            {
                // 非法功能码: 丢掉首字节重新同步(总线噪声)
                frame->erase(frame->begin());
                continue;
            }
        }

        // ---- 字节够了: CRC 校验 ----
        if (frame->size() >= total_needed)
        {
            if (verifyCrc16(*frame))
            {
                return true;
            }
            // CRC 失败: 丢掉首字节重新同步
            frame->erase(frame->begin());
            total_needed = 0;
        }
    }
}

// ============================== 轮询线程 ==============================

void TasSensorDriver::pollLoop()
{
    RCLCPP_INFO(get_logger(), "轮询线程启动");

    while (rclcpp::ok() && running_)
    {
        const auto cycle_start = std::chrono::steady_clock::now();

        // ---- 异常安全护栏: 任何一轮抛异常都只记录详细日志并继续下一轮,
        //      绝不让异常逃出线程函数(std::terminate 会拖死整个进程) ----
        try
        {
            // 一次完整轮询: 温湿度一帧 + CO2 一帧
            // 模拟模式走 queryRegistersSim(离线窗返回失败, 与真机被拔同构),
            // 真机走 queryRegisters(POSIX 同步收发); 二者之后共用同一套
            // "更新成员 + 发布"逻辑, 保证下游看到的数据形态完全一致。
            std::vector<uint16_t> values;

            // 每轮先清有效标志: 查询失败时 EnvData 会带"无效", 不会把
            // 断线前的旧值当新数据发布(否则下游会误以为传感器还活着)。
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                temp_hum_valid_ = false;
                co2_valid_ = false;
            }

            const bool hum_ok = sim_mode_ ?
                queryRegistersSim(kRegHumidity, 2, &values) :
                queryRegisters(kRegHumidity, 2, &values);
            if (hum_ok)
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                humidity_ = static_cast<float>(values[0]) / 10.0f;
                // 温度是 16 位补码(0xFF9B = -10.1℃)
                temperature_ =
                    static_cast<float>(static_cast<int16_t>(values[1])) / 10.0f;
                temp_hum_valid_ = true;
            }

            if (sim_mode_ ?
                    queryRegistersSim(kRegCo2, 1, &values) :
                    queryRegisters(kRegCo2, 1, &values))
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                co2_ = static_cast<float>(values[0]);
                co2_valid_ = true;
            }

            // ---- 更新时刻/计数并发布 ----
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                last_update_ = now();
                ++cycle_count_;
            }
            // ---- 恢复日志(边沿触发): 上一轮失败而本轮任一查询成功,
            //      与真机路径的"串口已恢复"对齐, 供离线恢复观测 ----
            if (prev_cycle_failed_ && (temp_hum_valid_ || co2_valid_))
            {
                RCLCPP_INFO(
                    get_logger(), "%s",
                    sim_mode_ ? "模拟设备重新在位, 串口已恢复" : "串口已恢复");
            }
            prev_cycle_failed_ = !(temp_hum_valid_ || co2_valid_);

            // 离线语义: 本轮两个查询都失败(设备缺席/串口断)时不发布,
            // /sensor/env 静默 => 下游(面板/状态)按"无数据=离线"处理,
            // 而不是把"无效标志的数据"误当作存活传感器。
            if (temp_hum_valid_ || co2_valid_)
            {
                publishEnv();
            }

            if (show_logger_.load())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "第%llu轮: 温度=%.1f℃(%s) 湿度=%.1f%%RH(%s) CO2=%.0fppm(%s)",
                    static_cast<unsigned long long>(cycle_count_),
                    temperature_, temp_hum_valid_ ? "有效" : "无效",
                    humidity_, temp_hum_valid_ ? "有效" : "无效",
                    co2_, co2_valid_ ? "有效" : "无效");
            }

            // ---- 服务插队: 一轮结束后通知等待的查询服务 ----
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                if (force_query_)
                {
                    force_query_ = false;
                    query_cv_.notify_all();
                }
            }
        }
        catch (const std::exception & e)
        {
            // 已知异常(std::exception 及其子类): 记录类型+内容+轮次,
            // 本轮按失败处理, 继续下一轮 —— 传感器侧单轮失败无致命影响。
            RCLCPP_ERROR(
                get_logger(),
                "轮询第%llu轮异常: %s(%s), 本轮按无效数据处理, 继续轮询",
                static_cast<unsigned long long>(cycle_count_),
                e.what(), typeid(e).name());
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                temp_hum_valid_ = false;
                co2_valid_ = false;
            }
        }
        catch (...)
        {
            // 未知异常(非 std::exception): 同样不退出, 记录后继续。
            RCLCPP_ERROR(
                get_logger(),
                "轮询第%llu轮发生未知异常(非 std::exception), "
                "本轮按无效数据处理, 继续轮询",
                static_cast<unsigned long long>(cycle_count_));
            {
                std::lock_guard<std::mutex> lock(reply_mutex_);
                temp_hum_valid_ = false;
                co2_valid_ = false;
            }
        }

        // ---- 睡满剩余时间(凑足一个轮询周期; 异常轮同样保证节奏) ----
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - cycle_start).count();
        const int sleep_ms = std::max(
            50, poll_interval_ms_.load() - static_cast<int>(elapsed));
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    RCLCPP_INFO(get_logger(), "轮询线程退出");
}

// ============================== 单次问询(同步收发) ==============================

bool TasSensorDriver::queryRegisters(
    uint16_t reg, uint16_t count, std::vector<uint16_t> * values)
{
    // 保证与上一次问询间隔 ≥ inter_query_gap_ms(传感器要求 ≥200ms)
    std::this_thread::sleep_until(
        last_query_time_ + std::chrono::milliseconds(inter_query_gap_ms_.load()));

    // 串口未打开: 尝试打开(设备插回后在这里自动恢复)
    if (fd_ < 0)
    {
        if (!openSerial())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "串口打开失败(%s): %s, 下轮继续尝试",
                device_name_.c_str(), std::strerror(errno));
            return false;
        }
        RCLCPP_INFO(get_logger(), "串口已恢复");
    }

    // 组帧发送(自动附 CRC)
    const std::vector<uint8_t> frame =
        buildReadRequest(static_cast<uint8_t>(modbus_addr_), reg, count);
    if (!sendFrame(frame))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "发送问询失败: %s, 关闭串口下轮重试", std::strerror(errno));
        closeSerial();
        return false;
    }
    last_query_time_ = std::chrono::steady_clock::now();

    // 同步等应答(内部带总超时, 超时自动清输入缓冲)
    std::vector<uint8_t> reply;
    const int timeout_ms = query_timeout_ms_.load();
    errno = 0;
    if (!receiveFrame(&reply, timeout_ms, static_cast<uint8_t>(modbus_addr_)))
    {
        // 区分超时与设备错误: errno 置位说明是读错误(设备被拔)
        const int saved_errno = errno;
        if (fd_ >= 0 && (saved_errno == EIO || saved_errno == EBADF))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "读取失败: %s, 关闭串口下轮重试", std::strerror(saved_errno));
            closeSerial();
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "问询超时(%dms), 已清空输入缓冲", timeout_ms);
        }
        return false;
    }

    // 解析应答(地址/功能码/长度/CRC 全对才算成功)
    return parseReadResponse(reply, static_cast<uint8_t>(modbus_addr_), values);
}

// ============================== 模拟模式 ==============================

bool TasSensorDriver::simDevicePresent() const
{
    const double elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - sim_start_time_).count();

    // 初始离线窗: [0, initial_absent)
    if (elapsed < sim_initial_absent_sec_)
    {
        return false;
    }
    // 中途离线窗: 从 disconnect_at 起, 每 repeat_period 重复一次,
    // 每次持续 disconnect_duration(<=0 表示从不模拟中途断连)
    if (sim_disconnect_at_sec_ <= 0.0 || elapsed < sim_disconnect_at_sec_)
    {
        return true;
    }
    if (sim_disconnect_repeat_period_sec_ > 0.0)
    {
        const double phase = std::fmod(
            elapsed - sim_disconnect_at_sec_, sim_disconnect_repeat_period_sec_);
        return phase >= sim_disconnect_duration_sec_;
    }
    return elapsed >= sim_disconnect_at_sec_ + sim_disconnect_duration_sec_;
}

bool TasSensorDriver::queryRegistersSim(
    uint16_t reg, uint16_t count, std::vector<uint16_t> * values)
{
    if (values == nullptr)
    {
        return false;
    }
    // 模拟离线: 与真机"问询超时/无应答"返回 false 一致,
    // 由 pollLoop 清 valid 标志并发布"无效"数据, 下游可感知。
    if (!simDevicePresent())
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 15000,
            "模拟传感器离线(等待接入), 发布数据将标记为无效");
        return false;
    }

    // 生成"当前读数": 缓慢正弦 + 小抖动, 接近真实环境曲线;
    // 抖动相位带 reg 偏移, 让温湿度/CO2 两次问询略有差异(更像真机)。
    const double elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - sim_start_time_).count();
    const double phase = elapsed + 0.37 * static_cast<double>(reg);

    values->clear();
    switch (reg)
    {
        case kRegHumidity:
            // 寄存器 0x0000 起两个: [湿度 raw, 温度 raw(16位补码)]
            {
                const double hum = sim_base_humidity_ +
                                   5.0 * std::sin(elapsed / 23.0) +
                                   0.8 * std::sin(phase * 7.3);
                const double temp = sim_base_temperature_ +
                                    2.0 * std::sin(elapsed / 41.0) +
                                    0.3 * std::sin(phase * 5.1);
                const uint16_t hum_raw = static_cast<uint16_t>(
                    std::clamp(hum, 0.0, 100.0) * 10.0);
                const int16_t temp_raw = static_cast<int16_t>(
                    std::clamp(temp, -40.0, 85.0) * 10.0);
                values->push_back(hum_raw);
                values->push_back(static_cast<uint16_t>(temp_raw));
            }
            break;
        case kRegCo2:
            // 寄存器 0x0005 一个: CO2 ppm 原值
            {
                const double co2 = sim_base_co2_ +
                                   120.0 * std::sin(elapsed / 17.0) +
                                   15.0 * std::sin(phase * 3.7);
                const uint16_t co2_raw = static_cast<uint16_t>(
                    std::clamp(co2, 350.0, 5000.0));
                values->push_back(co2_raw);
            }
            break;
        default:
            RCLCPP_WARN(
                get_logger(),
                "模拟模式: 未支持的寄存器地址 0x%04X (count=%u), 返回失败",
                reg, count);
            return false;
    }
    return true;
}

// ============================== 发布与服务 ==============================

void TasSensorDriver::publishEnv()
{
    sensor_interfaces::msg::EnvData msg;
    {
        std::lock_guard<std::mutex> lock(reply_mutex_);
        msg.header.stamp = last_update_;
        msg.header.frame_id = "env_sensor";
        msg.temp_hum_valid = temp_hum_valid_;
        msg.co2_valid = co2_valid_;
        msg.temperature = temperature_;
        msg.humidity = humidity_;
        msg.co2 = co2_;
    }
    env_pub_->publish(msg);
}

void TasSensorDriver::queryEnvCallback(
    const sensor_interfaces::srv::QueryEnv::Request::SharedPtr /*request*/,
    const sensor_interfaces::srv::QueryEnv::Response::SharedPtr response)
{
    RCLCPP_INFO(get_logger(), "收到查询请求, 插队执行一轮完整查询...");

    // 通知轮询线程尽快完成一轮查询
    {
        std::lock_guard<std::mutex> lock(reply_mutex_);
        force_query_ = true;
    }

    // 等待该轮结束(轮询线程完成一轮后会清掉 force_query_ 并唤醒这里)
    std::unique_lock<std::mutex> lock(reply_mutex_);
    query_cv_.wait_for(
        lock, std::chrono::milliseconds(poll_interval_ms_.load() + 3000),
        [this]()
        {
            return !force_query_;
        });

    // 返回最新数据
    response->success = temp_hum_valid_ || co2_valid_;
    response->header.stamp = last_update_;
    response->temperature = temperature_;
    response->humidity = humidity_;
    response->co2 = co2_;
    response->message = response->success ?
        "ok" : "传感器无有效数据, 请检查供电/接线/Modbus地址";

    RCLCPP_INFO(
        get_logger(), "查询完成: success=%d 温度=%.1f 湿度=%.1f CO2=%.0f",
        response->success, response->temperature,
        response->humidity, response->co2);
}

// ============================== 动态参数 ==============================

rcl_interfaces::msg::SetParametersResult TasSensorDriver::onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params)
    {
        if (p.get_name() == "poll_interval_ms")
        {
            poll_interval_ms_.store(std::max(500, static_cast<int>(p.as_int())));
        }
        else if (p.get_name() == "query_timeout_ms")
        {
            query_timeout_ms_.store(std::max(200, static_cast<int>(p.as_int())));
        }
        else if (p.get_name() == "inter_query_gap_ms")
        {
            inter_query_gap_ms_.store(std::max(200, static_cast<int>(p.as_int())));
        }
        else if (p.get_name() == "show_logger")
        {
            show_logger_.store(p.as_bool());
        }
        else
        {
            result.successful = false;
            result.reason = "参数 '" + p.get_name() + "' 只读, 需重启";
        }
    }
    return result;
}

}  // namespace tas_sensor_driver

// ============================== 组件注册 ==============================
// 既能独立运行, 也能被 composition 容器加载。

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tas_sensor_driver::TasSensorDriver)
