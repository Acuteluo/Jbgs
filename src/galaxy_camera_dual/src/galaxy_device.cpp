// Copyright (c) 2024, galaxy_camera_dual contributors
//
// 文件: galaxy_device.cpp
// 作用: GalaxyDevice(大恒 Galaxy SDK 的线程安全封装)的实现。
//
// 实现要点:
//  - 两把全局互斥锁 + 每设备一把互斥锁;
//  - 加锁顺序固定为: mutex_(设备锁) -> g_list_mutex(列表锁), 任何路径
//    都不会反向加锁, 因此不会死锁;
//  - 所有 SDK 返回值都经 GX_SUCCESS 宏统一判断;
//  - 【重要】GxIAPI 部分接口在异常场景下(如 GigE 掉线)会直接抛 C++
//    异常(实测 "send data failed" 从 GXGetImage 内部抛出), 因此这里
//    所有 SDK 调用都包了一层 try/catch, 异常一律转换成错误码返回,
//    让上层节点走"失败计数 -> 自动重连"的正常恢复路径, 而不是崩溃。
#include "galaxy_camera_dual/galaxy_device.hpp"

#include <vector>

/// 判断 SDK 状态码是否成功(0 为成功, 其余为错误码)。
#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

namespace galaxy_camera_dual
{

// ----------------------------------------------------------------------
// 进程级全局状态(仅本文件可见)
// ----------------------------------------------------------------------
namespace
{
/// 保护 GXInitLib/GXCloseLib 的互斥锁。
/// 原因: GXInitLib/GXCloseLib 是进程级全局调用, 不能并发执行。
std::mutex g_lib_mutex;

/// 库的引用计数。>0 表示库已初始化; 减到 0 时才真正执行 GXCloseLib()。
/// 这样多个节点实例/多台设备可以安全地各自调用 init/close。
int g_lib_refcount = 0;

/// 保护"设备列表"的互斥锁。
/// 原因: GXUpdateAllDeviceList / GXGetAllDeviceBaseInfo / GXGetDeviceIPInfo
/// 都会改写 SDK 内部的设备列表, 而 open() 与 listDevices() 可能并发执行,
/// 必须串行化。
std::mutex g_list_mutex;

/// 构造 GXOpenDevice 所需的打开参数结构体。
/// content : 与打开方式对应的内容(SN / UserID / IP / 序号字符串);
/// mode    : 打开方式(GX_OPEN_SN / GX_OPEN_USERID / GX_OPEN_IP / GX_OPEN_INDEX);
/// 访问模式固定用 GX_ACCESS_CONTROL(控制模式): 既能读写参数、又能取流,
/// 同时还允许其它客户端(如大恒的调试工具)同时连接, 便于现场排查。
GX_OPEN_PARAM makeOpenParam(const std::string & content, GX_OPEN_MODE_CMD mode)
{
    GX_OPEN_PARAM param;
    param.pszContent = const_cast<char *>(content.c_str());
    param.openMode = mode;
    param.accessMode = GX_ACCESS_CONTROL;
    return param;
}
}  // namespace

// ----------------------------------------------------------------------
// 库生命周期
// ----------------------------------------------------------------------

bool GalaxyDevice::initLibrary()
{
    std::lock_guard<std::mutex> lock(g_lib_mutex);

    // 库已初始化: 只增加引用计数, 不重复调用 GXInitLib。
    if (g_lib_refcount > 0)
    {
        ++g_lib_refcount;
        return true;
    }

    // 首次初始化: 真正调用 GXInitLib()(包异常)。
    try
    {
        if (GX_SUCCESS(GXInitLib()))
        {
            g_lib_refcount = 1;
            return true;
        }
    }
    catch (const std::exception &)
    {
        return false;
    }
    return false;
}

void GalaxyDevice::closeLibrary()
{
    std::lock_guard<std::mutex> lock(g_lib_mutex);

    // 从未初始化成功过, 无需关闭。
    if (g_lib_refcount <= 0)
    {
        return;
    }

    // 引用计数减到 0 才真正关闭, 避免影响进程里其它还在使用库的对象。
    --g_lib_refcount;
    if (g_lib_refcount == 0)
    {
        try
        {
            GXCloseLib();
        }
        catch (const std::exception &)
        {
            // 关闭失败也无可奈何, 进程即将退出
        }
    }
}

bool GalaxyDevice::isLibraryInited()
{
    std::lock_guard<std::mutex> lock(g_lib_mutex);
    return g_lib_refcount > 0;
}

std::vector<DeviceInfo> GalaxyDevice::listDevices(uint32_t timeout_ms)
{
    std::vector<DeviceInfo> result;
    std::lock_guard<std::mutex> lock(g_list_mutex);

    try
    {
        // 第一步: 刷新 SDK 内部设备列表并拿到设备个数。
        // 用 GXUpdateAllDeviceList 而不是 GXUpdateDeviceList:
        // 前者能枚举所有网段的 GigE 相机(后者只能枚举同网段)。
        uint32_t count = 0;
        if (!GX_SUCCESS(GXUpdateAllDeviceList(&count, timeout_ms)) || count == 0)
        {
            return result;
        }

        // 第二步: 批量取出所有设备的基础信息(型号/SN/UserID 等)。
        std::vector<GX_DEVICE_BASE_INFO> base_info(count);
        size_t buffer_size = count * sizeof(GX_DEVICE_BASE_INFO);
        if (!GX_SUCCESS(GXGetAllDeviceBaseInfo(base_info.data(), &buffer_size)))
        {
            return result;
        }

        // 第三步: 逐台补充网络信息(IP/MAC), 转成 DeviceInfo 返回。
        result.reserve(count);
        for (uint32_t i = 0; i < count; ++i)
        {
            DeviceInfo info;
            info.index = i + 1;   // GxIAPI 的设备序号从 1 开始
            info.vendor_name = base_info[i].szVendorName;
            info.model_name = base_info[i].szModelName;
            info.serial_number = base_info[i].szSN;
            info.user_id = base_info[i].szUserID;

            GX_DEVICE_IP_INFO ip_info = {};
            if (GX_SUCCESS(GXGetDeviceIPInfo(info.index, &ip_info)))
            {
                info.ip = ip_info.szIP;
                info.mac = ip_info.szMAC;
            }
            result.push_back(info);
        }
    }
    catch (const std::exception &)
    {
        result.clear();
    }
    return result;
}

// ----------------------------------------------------------------------
// 打开 / 关闭
// ----------------------------------------------------------------------

GalaxyDevice::~GalaxyDevice()
{
    // 对象销毁时兜底关闭设备, 防止句柄泄漏。
    close();
}

bool GalaxyDevice::open(const DeviceAddress & address, std::string * error_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 已打开则拒绝重复打开(防止覆盖已有句柄造成泄漏)。
    if (handle_ != nullptr)
    {
        if (error_msg != nullptr)
        {
            *error_msg = "device already open";
        }
        return false;
    }

    GX_STATUS status = GX_STATUS_ERROR;
    try
    {
        status = openInternal(address);
    }
    catch (const std::exception & ex)
    {
        if (error_msg != nullptr)
        {
            *error_msg = std::string("open exception: ") + ex.what();
        }
        return false;
    }

    if (!GX_SUCCESS(status))
    {
        if (error_msg != nullptr)
        {
            *error_msg = "open failed, status = " + std::to_string(status);
        }
        return false;
    }
    return true;
}

GX_STATUS GalaxyDevice::openInternal(const DeviceAddress & address)
{
    // 注意: 调用方必须已持有 mutex_。
    //
    // GxIAPI 要求 GXOpenDevice 之前先刷新内部设备列表, 否则可能返回
    // GX_STATUS_NOT_FOUND_DEVICE。刷列表会改写 SDK 全局状态, 因此临时
    // 拿一下 g_list_mutex(锁顺序固定为: mutex_ -> g_list_mutex)。
    {
        std::lock_guard<std::mutex> lock(g_list_mutex);
        uint32_t count = 0;
        GXUpdateAllDeviceList(&count, 500);
    }

    GX_STATUS status = GX_STATUS_ERROR;
    GX_DEV_HANDLE handle = nullptr;

    // 按固定优先级选择打开方式: 序列号 > UserID > IP > 设备序号。
    if (!address.serial_number.empty())
    {
        GX_OPEN_PARAM param = makeOpenParam(address.serial_number, GX_OPEN_SN);
        status = GXOpenDevice(&param, &handle);
    }
    else if (!address.user_id.empty())
    {
        GX_OPEN_PARAM param = makeOpenParam(address.user_id, GX_OPEN_USERID);
        status = GXOpenDevice(&param, &handle);
    }
    else if (!address.ip_address.empty())
    {
        GX_OPEN_PARAM param = makeOpenParam(address.ip_address, GX_OPEN_IP);
        status = GXOpenDevice(&param, &handle);
    }
    else if (address.device_index > 0)
    {
        // 按设备序号打开: 与 SN/IP/UserID 一样走标准 GXOpenDevice 接口,
        // 访问模式统一为 GX_ACCESS_CONTROL(控制模式)。
        // 注意: 不能用已弃用的 GXOpenDeviceByIndex——它默认独占模式;
        // GX_OPEN_INDEX 模式下 pszContent 填序号字符串, 序号从 1 开始。
        GX_OPEN_PARAM param =
            makeOpenParam(std::to_string(address.device_index), GX_OPEN_INDEX);
        status = GXOpenDevice(&param, &handle);
    }
    else
    {
        // 四种寻址方式都没有提供有效值, 直接返回参数错误。
        // (常见诱因: 节点名与参数文件命名空间不匹配导致参数全为空)
        return GX_STATUS_INVALID_PARAMETER;
    }

    // 打开成功才把句柄保存到成员变量。
    if (GX_SUCCESS(status))
    {
        handle_ = handle;
    }
    return status;
}

void GalaxyDevice::close()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 未打开(或已关闭)则直接返回, 保证幂等。
    if (handle_ == nullptr)
    {
        return;
    }

    try
    {
        // 先停止采集: 这一步会唤醒正阻塞在 GXGetImage 里的采集线程,
        // 随后 GXCloseDevice 才安全。顺序不能反。
        GXSendCommand(
            handle_,
            static_cast<GX_FEATURE_ID_CMD>(GX_COMMAND_ACQUISITION_STOP));
        GXCloseDevice(handle_);
    }
    catch (const std::exception &)
    {
        // 设备已掉线时这些调用可能抛异常, 吞掉即可
    }
    handle_ = nullptr;
}

bool GalaxyDevice::isOpen() const
{
    return handle_ != nullptr;
}

// ----------------------------------------------------------------------
// 特性读写
// 每个函数结构完全一致: 锁 -> 判空 -> try/catch 调 SDK -> 返回布尔
// ----------------------------------------------------------------------

bool GalaxyDevice::getInt(GX_FEATURE_ID_CMD id, int64_t * value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr || value == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXGetInt(handle_, id, value));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::sendCommand(GX_FEATURE_ID_CMD id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXSendCommand(handle_, id));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::setBool(GX_FEATURE_ID_CMD id, bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXSetBool(handle_, id, value));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::setInt(GX_FEATURE_ID_CMD id, int64_t value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXSetInt(handle_, id, value));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::getFloat(GX_FEATURE_ID_CMD id, double * value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr || value == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXGetFloat(handle_, id, value));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::setFloat(GX_FEATURE_ID_CMD id, double value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXSetFloat(handle_, id, value));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::getFloatRange(GX_FEATURE_ID_CMD id, GX_FLOAT_RANGE * range)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr || range == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXGetFloatRange(handle_, id, range));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::setEnum(GX_FEATURE_ID_CMD id, int64_t value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXSetEnum(handle_, id, value));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::getString(GX_FEATURE_ID_CMD id, std::string * value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr || value == nullptr)
    {
        return false;
    }

    try
    {
        // SDK 规定: 读取字符串前必须先用 GXGetStringLength 拿长度。
        size_t length = 0;
        if (!GX_SUCCESS(GXGetStringLength(handle_, id, &length)))
        {
            return false;
        }

        // 多申请 2 字节(1 字节冗余 + 1 字节结束符), 防止越界。
        std::vector<char> buffer(length + 2);
        size_t size = buffer.size();
        if (!GX_SUCCESS(GXGetString(handle_, id, buffer.data(), &size)))
        {
            return false;
        }
        buffer.back() = '\0';   // 兜底保证 C 字符串结束符存在
        value->assign(buffer.data());
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::setString(GX_FEATURE_ID_CMD id, const std::string & value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        // SDK 的 GXSetString 签名是 char*(非 const), 需要转换。
        return GX_SUCCESS(
            GXSetString(handle_, id, const_cast<char *>(value.c_str())));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::isImplemented(GX_FEATURE_ID_CMD id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        bool implemented = false;
        if (!GX_SUCCESS(GXIsImplemented(handle_, id, &implemented)))
        {
            return false;
        }
        return implemented;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::setAcquisitionBufferNumber(uint64_t buffer_count)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        // 注意: "GXSetAcqusitionBufferNumber" 是 SDK 自带的拼写(Acqusition),
        // 不是我们的笔误, 必须按 SDK 原名调用。
        return GX_SUCCESS(GXSetAcqusitionBufferNumber(handle_, buffer_count));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool GalaxyDevice::startAcquisition()
{
    return sendCommand(
        static_cast<GX_FEATURE_ID_CMD>(GX_COMMAND_ACQUISITION_START));
}

bool GalaxyDevice::stopAcquisition()
{
    return sendCommand(
        static_cast<GX_FEATURE_ID_CMD>(GX_COMMAND_ACQUISITION_STOP));
}

bool GalaxyDevice::flushQueue()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (handle_ == nullptr)
    {
        return false;
    }
    try
    {
        return GX_SUCCESS(GXFlushQueue(handle_));
    }
    catch (const std::exception &)
    {
        return false;
    }
}

GX_STATUS GalaxyDevice::grab(GX_FRAME_DATA * frame_data, uint32_t timeout_ms)
{
    // 只在取句柄时加锁, 阻塞等待期间不持锁 —— 否则软触发线程的
    // sendCommand 会被阻塞到取图超时, 触发丢失、帧率掉坑(实测教训)。
    GX_DEV_HANDLE h = nullptr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        h = handle_;
    }
    if (h == nullptr || frame_data == nullptr)
    {
        return GX_STATUS_INVALID_HANDLE;
    }

    try
    {
        // 相机掉线时 SDK 内部可能直接抛异常(实测 "send data failed"),
        // 这里转成错误码返回, 让上层走失败重连路径。
        return GXGetImage(h, frame_data, timeout_ms);
    }
    catch (const std::exception &)
    {
        return GX_STATUS_ERROR;
    }
}

std::string GalaxyDevice::describe()
{
    // 拼一行设备摘要用于日志; 各字段读取失败时自动留空, 不影响流程。
    std::string model;
    std::string serial;
    std::string user_id;
    std::string fw;        // 固件版本(不同批次可能不同, 排查"同型号不同行为"的关键)
    std::string dev_ver;   // 设备版本

    getString(GX_STRING_DEVICE_MODEL_NAME, &model);
    getString(GX_STRING_DEVICE_SERIAL_NUMBER, &serial);
    getString(GX_STRING_DEVICE_USERID, &user_id);
    getString(GX_STRING_DEVICE_FIRMWARE_VERSION, &fw);
    getString(GX_STRING_DEVICE_VERSION, &dev_ver);

    return "model=" + model + ", SN=" + serial + ", UserID=" + user_id +
           ", FW=" + fw + ", Ver=" + dev_ver;
}

}  // namespace galaxy_camera_dual
