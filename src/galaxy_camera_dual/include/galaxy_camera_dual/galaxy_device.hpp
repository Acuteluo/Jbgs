// Copyright (c) 2024, galaxy_camera_dual contributors
//
// 文件: galaxy_device.hpp
// 作用: 大恒 Galaxy SDK(GxIAPI) 的 C++ 线程安全封装层 —— 声明部分。
//
// 为什么要加这一层(而不是像原包那样直接调用 GxIAPI):
//   1. GxIAPI 是纯 C 接口, 直接调用时代码零散、容易漏掉错误处理;
//   2. GXInitLib()/GXCloseLib() 是"进程级"全局调用: 整个进程只能有一对
//      完整的 初始化 -> 关闭 生命周期。当同一进程里有多个节点实例
//      (如 composition 容器)或多个相机设备时, 各自直接调用会互相干扰,
//      因此这里用"引用计数 + 互斥锁"统一管理;
//   3. 同一台相机的 GX_DEV_HANDLE 会被两个线程同时访问:
//      - 采集线程里阻塞式等待图像的 GXGetImage();
//      - ROS 执行器线程里响应参数回调的 GXSetFloat() 等。
//      GxIAPI 不保证这种并发安全, 所以给每个设备配一把互斥锁,
//      把所有 SDK 调用按设备串行化, 从根上消除竞态;
//   4. 两台同型号相机必须能按 序列号/UserID/IP/设备序号 分别寻址,
//      统一封装为 DeviceAddress, 优先级固定, 逻辑集中在一处。
#pragma once

#include "GxIAPI.h"   // 大恒 Galaxy SDK 主头文件

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace galaxy_camera_dual
{

/// 相机寻址方式。
/// 打开时按以下优先级选择: serial_number > user_id > ip_address > device_index。
/// 强烈建议使用序列号: 两台同型号相机按"序号"寻址时, 枚举顺序重启后
/// 可能互换, 会导致左右相机错位; 序列号出厂唯一且固定。
struct DeviceAddress
{
    std::string serial_number;   ///< 相机序列号(推荐, 唯一且稳定)
    std::string user_id;         ///< 用户自定义 ID(可用 user_id_to_set 参数烧写)
    std::string ip_address;      ///< 相机 IP 地址
    uint32_t device_index = 0;   ///< 设备序号, 从 1 开始; 0 表示不使用
};

/// 枚举到的一台设备的信息, 用于启动日志打印, 帮助用户区分左右相机。
struct DeviceInfo
{
    uint32_t index = 0;          ///< 设备序号, 从 1 开始
    std::string vendor_name;     ///< 厂商名
    std::string model_name;      ///< 型号, 如 MER-500-14GC
    std::string serial_number;   ///< 序列号
    std::string user_id;         ///< 用户自定义 ID
    std::string ip;              ///< IP 地址(仅 GigE 有效)
    std::string mac;             ///< MAC 地址(仅 GigE 有效)
};

/// 单台相机的线程安全封装。
///
/// 线程模型约定:
///  - 除构造函数/析构函数外, 所有成员函数内部都先锁 mutex_ 再调用 SDK,
///    因此任意线程(采集线程、ROS 执行器线程)可随时调用, 互不冲突;
///  - grab() 在等待图像期间会一直持有 mutex_, 最长阻塞 timeout_ms。
///    因此参数回调(如设曝光)最坏情况下会等待这一个超时周期, 属正常现象;
///  - 锁的获取顺序固定为 mutex_ -> g_list_mutex(见 .cpp), 不会出现
///    反向加锁, 从设计上杜绝死锁;
///  - 本类禁止拷贝: 一个设备句柄对应一个对象, 拷贝会造成重复关闭。
class GalaxyDevice
{
public:
    // ------------------------------------------------------------------
    // 进程级库生命周期(静态方法, 引用计数, 线程安全)
    // ------------------------------------------------------------------

    /// 初始化 GxIAPI 库。
    /// 第一次调用时真正执行 GXInitLib(); 后续调用只把引用计数加 1。
    /// 每次成功调用都必须与一次 closeLibrary() 配对。
    static bool initLibrary();

    /// 关闭 GxIAPI 库。引用计数减 1, 减到 0 时才真正执行 GXCloseLib()。
    /// 这样进程里其它还在使用库的对象不会受影响。
    static void closeLibrary();

    /// 库当前是否已初始化(引用计数 > 0)。
    static bool isLibraryInited();

    /// 枚举所有可到达的设备。
    /// 内部使用 GXUpdateAllDeviceList: 相比 GXUpdateDeviceList,
    /// 它能枚举其它网段里的 GigE 相机。失败或没有设备时返回空列表。
    static std::vector<DeviceInfo> listDevices(uint32_t timeout_ms = 1000);

    GalaxyDevice() = default;

    /// 析构为虚函数: 允许 SimulatedGalaxyDevice(模拟设备, 见
    /// simulated_galaxy_device.hpp)继承本类, 通过基类指针多态使用。
    virtual ~GalaxyDevice();

    // 禁止拷贝: 一个设备句柄对应一个对象, 拷贝会造成重复关闭。
    GalaxyDevice(const GalaxyDevice &) = delete;
    GalaxyDevice & operator=(const GalaxyDevice &) = delete;

    /// 打开设备。
    /// 内部先刷新 SDK 设备列表(GXOpenDevice 的前置要求), 再按
    /// DeviceAddress 的优先级尝试打开。失败时返回 false 并把原因写入
    /// error_msg(可为 nullptr)。
    virtual bool open(const DeviceAddress & address, std::string * error_msg = nullptr);

    /// 关闭设备(先停止采集再关闭句柄)。可重复调用, 幂等。
    virtual void close();

    /// 设备当前是否处于打开状态。
    virtual bool isOpen() const;

    // ------------------------------------------------------------------
    // 以下所有设备操作都是线程安全的(内部互斥锁), 可与采集线程里的
    // grab() 并发调用, 例如参数回调线程设置曝光/增益。
    // 声明为 virtual 的原因同上: 供模拟设备类(SimulatedGalaxyDevice)
    // 覆写, 节点代码无需区分真机/模拟即可运行与热插拔测试。
    // ------------------------------------------------------------------

    /// 读取整型特性, 如 GX_INT_PAYLOAD_SIZE。
    virtual bool getInt(GX_FEATURE_ID_CMD id, int64_t * value);

    /// 写入整型特性, 如 GX_DEV_INT_COMMAND_TIMEOUT。
    virtual bool setInt(GX_FEATURE_ID_CMD id, int64_t value);

    /// 读取浮点特性, 如 GX_FLOAT_EXPOSURE_TIME。
    virtual bool getFloat(GX_FEATURE_ID_CMD id, double * value);

    /// 写入浮点特性, 如曝光、增益。
    virtual bool setFloat(GX_FEATURE_ID_CMD id, double value);
    virtual bool setBool(GX_FEATURE_ID_CMD id, bool value);
    virtual bool sendCommand(GX_FEATURE_ID_CMD id);

    /// 读取浮点特性的取值范围(最小值/最大值/步长)。
    virtual bool getFloatRange(GX_FEATURE_ID_CMD id, GX_FLOAT_RANGE * range);

    /// 写入枚举特性, 如 GX_ENUM_ACQUISITION_FRAME_RATE_MODE。
    virtual bool setEnum(GX_FEATURE_ID_CMD id, int64_t value);

    /// 读取字符串特性, 如序列号、型号、UserID。
    virtual bool getString(GX_FEATURE_ID_CMD id, std::string * value);

    /// 写入字符串特性, 如把 UserID 烧写进相机。
    virtual bool setString(GX_FEATURE_ID_CMD id, const std::string & value);

    /// 查询某个特性是否被当前相机支持(型号不同, 特性可能缺失)。
    virtual bool isImplemented(GX_FEATURE_ID_CMD id);

    /// 设置 SDK 内部的采集缓冲队列个数(对 GigE 稳定性有帮助)。
    virtual bool setAcquisitionBufferNumber(uint64_t buffer_count);

    /// 开始采集(GX_COMMAND_ACQUISITION_START)。
    virtual bool startAcquisition();

    /// 停止采集(GX_COMMAND_ACQUISITION_STOP)。
    /// 注意: 它同时会唤醒正阻塞在 grab() 里的线程, 是优雅关机的关键一步。
    virtual bool stopAcquisition();

    /// 清空 SDK 内部尚未取走的图像队列。
    virtual bool flushQueue();

    /// 阻塞式取一帧图像。
    /// 调用方必须预先提供 frame_data->pImgBuf, 且大小 >= 载荷大小
    /// (GX_INT_PAYLOAD_SIZE), 否则 SDK 返回 GX_STATUS_NEED_MORE_BUFFER。
    /// 返回原始 GX_STATUS; 返回 GX_STATUS_SUCCESS 时还需再检查
    /// frame_data->nStatus == GX_FRAME_STATUS_SUCCESS(排除残帧)。
    virtual GX_STATUS grab(GX_FRAME_DATA * frame_data, uint32_t timeout_ms);

    /// 一行设备摘要(型号/序列号/UserID), 用于日志。
    virtual std::string describe();

private:
    /// 实际执行打开的私有函数(调用方必须已持有 mutex_)。
    GX_STATUS openInternal(const DeviceAddress & address);

    GX_DEV_HANDLE handle_ = nullptr;   ///< SDK 设备句柄, nullptr 表示未打开
    std::mutex mutex_;                 ///< 本设备的互斥锁, 串行化所有 SDK 调用
};

}  // namespace galaxy_camera_dual
