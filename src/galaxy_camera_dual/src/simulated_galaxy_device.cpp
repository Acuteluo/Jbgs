// Copyright (c) 2024, galaxy_camera_dual contributors
//
// 文件: simulated_galaxy_device.cpp
// 作用: SimulatedGalaxyDevice(模拟大恒相机)的实现。
//
// 实现说明:
//  - 时间窗判定以"设备对象创建时刻"为基准(start_time_), 与节点的启动
//    时刻基本一致, 便于按 launch 参数精确复现"初始断连/中途断连";
//  - grab() 严格按目标帧率节拍出图(而不是一次 sleep 到超时), 让
//    ros2 topic hz 看到的曲线与真机一致;
//  - 所有"设置类"操作(setFloat/setEnum/...)一律成功 —— 真机上这些
//    操作只影响相机内部状态, 与热插拔行为无关, 模拟无必要逐项仿真。

#include "galaxy_camera_dual/simulated_galaxy_device.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <thread>

namespace galaxy_camera_dual
{

namespace
{
/// 模拟设备的 tick 频率(1MHz): 供节点把 nTimestamp 换算成 ROS 时间,
/// 数值只需自洽, 与真机晶振无关。
constexpr int64_t kSimTickFreqHz = 1000000;

/// 无实拍帧时的兜底帧率(与 MER-500-14GC 手册标称一致)。
constexpr double kDefaultSimFps = 14.0;

/// 把目录下全部 jpg 读进轮播列表, 缩放到模拟分辨率。
/// 目录不存在/无图时返回空列表, 由 produceFrame 退化为程序合成图。
std::vector<cv::Mat> loadPlaylist(
    const std::string & dir, const cv::Size & size)
{
    std::vector<cv::Mat> frames;
    std::error_code ec;
    if (!std::filesystem::exists(dir, ec))
    {
        return frames;
    }
    std::vector<std::filesystem::path> files;
    for (const auto & entry : std::filesystem::directory_iterator(dir))
    {
        // 只认 .jpg/.jpeg(大小写不限; 目录里现存的均为该格式)
        std::string lower = entry.path().extension().string();
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (lower == ".jpg" || lower == ".jpeg")
        {
            files.push_back(entry.path());
        }
    }
    // 文件名排序, 保证轮播顺序稳定可复现
    std::sort(files.begin(), files.end());
    for (const auto & path : files)
    {
        const cv::Mat img = cv::imread(path.string(), cv::IMREAD_COLOR);
        if (img.empty())
        {
            continue;
        }
        cv::Mat resized;
        cv::resize(img, resized, size, 0.0, 0.0, cv::INTER_AREA);
        frames.push_back(resized);
    }
    return frames;
}
}  // namespace

SimulatedGalaxyDevice::SimulatedGalaxyDevice(
    const std::string & name, const SimDeviceParams & params)
: name_(name),
  params_(params),
  start_time_(std::chrono::steady_clock::now())
{
    // 帧率兜底: 与真机标称一致, 便于无参直跑
    if (params_.frame_rate <= 0.0)
    {
        params_.frame_rate = kDefaultSimFps;
    }
    // 轮播帧懒加载: 放在首次 open 时(见 open()), 避免构造函数里做 IO
}

SimulatedGalaxyDevice::~SimulatedGalaxyDevice()
{
    close();
}

// ----------------------------------------------------------------------
// 在位判定: 三类时间窗并集 = 设备离线
//   [0, initial_absent_sec)                          初始离线
//   [disconnect_at_sec + k*period, +duration)        中途离线(可重复)
// ----------------------------------------------------------------------
bool SimulatedGalaxyDevice::inAbsentWindow(
    std::chrono::steady_clock::time_point t) const
{
    const double elapsed =
        std::chrono::duration<double>(t - start_time_).count();

    // 初始离线窗
    if (elapsed < params_.initial_absent_sec)
    {
        return true;
    }

    // 中途离线窗(disconnect_at_sec <= 0 表示不模拟中途断连)
    if (params_.disconnect_at_sec <= 0.0)
    {
        return false;
    }
    if (elapsed < params_.disconnect_at_sec)
    {
        return false;
    }
    if (params_.disconnect_repeat_period_sec > 0.0)
    {
        // 重复模式: 计算当前时刻落在第几个周期内, 再判是否在窗内
        const double since_first =
            elapsed - params_.disconnect_at_sec;
        const double phase =
            std::fmod(since_first, params_.disconnect_repeat_period_sec);
        return phase < params_.disconnect_duration_sec;
    }
    // 单次模式: 首个离线窗之后一直在位
    return elapsed <
           params_.disconnect_at_sec + params_.disconnect_duration_sec;
}

bool SimulatedGalaxyDevice::devicePresent() const
{
    return !inAbsentWindow(std::chrono::steady_clock::now());
}

// ----------------------------------------------------------------------
// 打开 / 关闭
// ----------------------------------------------------------------------
bool SimulatedGalaxyDevice::open(
    const DeviceAddress & /*address*/, std::string * error_msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (open_)
    {
        if (error_msg != nullptr)
        {
            *error_msg = "device already open";
        }
        return false;
    }

    // 离线窗内打开失败 —— 与真机"设备不在线, GXOpenDevice 找不到"一致
    if (!devicePresent())
    {
        if (error_msg != nullptr)
        {
            *error_msg =
                "simulated device absent (模拟设备离线, 等待接入)";
        }
        return false;
    }

    open_ = true;
    acquiring_ = false;
    frame_counter_ = 0;

    // 懒加载轮播帧(只加载一次)
    if (!playlist_ready_)
    {
        playlist_ = loadPlaylist(
            params_.image_dir, cv::Size(params_.sim_width, params_.sim_height));
        playlist_ready_ = true;
    }
    return true;
}

void SimulatedGalaxyDevice::close()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    open_ = false;
    acquiring_ = false;
}

bool SimulatedGalaxyDevice::isOpen() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

// ----------------------------------------------------------------------
// 特性读写(模拟: 记录/返回自洽值)
// ----------------------------------------------------------------------
bool SimulatedGalaxyDevice::getInt(GX_FEATURE_ID_CMD id, int64_t * value)
{
    if (value == nullptr)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!open_)
    {
        return false;
    }
    switch (id)
    {
        case GX_INT_PAYLOAD_SIZE:
            // RGB24 载荷 = 宽 x 高 x 3(节点据此分配取图缓冲)
            *value = static_cast<int64_t>(params_.sim_width) *
                     params_.sim_height * 3;
            return true;
        case GX_INT_TIMESTAMP_TICK_FREQUENCY:
            *value = kSimTickFreqHz;
            return true;
        default:
            return true;   // 其余整型特性一律"写入成功"
    }
}

bool SimulatedGalaxyDevice::setInt(GX_FEATURE_ID_CMD /*id*/, int64_t /*value*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::getFloat(GX_FEATURE_ID_CMD /*id*/, double * value)
{
    if (value == nullptr)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::setFloat(GX_FEATURE_ID_CMD /*id*/, double /*value*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::setBool(GX_FEATURE_ID_CMD /*id*/, bool /*value*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::getFloatRange(
    GX_FEATURE_ID_CMD /*id*/, GX_FLOAT_RANGE * /*range*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::setEnum(GX_FEATURE_ID_CMD /*id*/, int64_t /*value*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::getString(GX_FEATURE_ID_CMD id, std::string * value)
{
    if (value == nullptr)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!open_)
    {
        return false;
    }
    // describe() 会依次读这四个字符串, 给出可辨识的模拟值
    switch (id)
    {
        case GX_STRING_DEVICE_MODEL_NAME: *value = "SIM-MER-500-14GC"; return true;
        case GX_STRING_DEVICE_SERIAL_NUMBER: *value = "SIM-" + name_; return true;
        case GX_STRING_DEVICE_USERID: *value = name_; return true;
        case GX_STRING_DEVICE_FIRMWARE_VERSION: *value = "sim-1.0"; return true;
        case GX_STRING_DEVICE_VERSION: *value = "sim"; return true;
        default: *value = ""; return true;
    }
}

bool SimulatedGalaxyDevice::setString(
    GX_FEATURE_ID_CMD /*id*/, const std::string & /*value*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::isImplemented(GX_FEATURE_ID_CMD /*id*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::setAcquisitionBufferNumber(uint64_t /*buffer_count*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::sendCommand(GX_FEATURE_ID_CMD /*id*/)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

bool SimulatedGalaxyDevice::startAcquisition()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!open_)
    {
        return false;
    }
    acquiring_ = true;
    // 出图节拍从现在起算, 避免长时间等待后的首帧堆积
    next_frame_time_ = std::chrono::steady_clock::now();
    return true;
}

bool SimulatedGalaxyDevice::stopAcquisition()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    acquiring_ = false;
    return true;
}

bool SimulatedGalaxyDevice::flushQueue()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return open_;
}

// ----------------------------------------------------------------------
// 帧生成
// ----------------------------------------------------------------------
void SimulatedGalaxyDevice::produceFrame(cv::Mat & rgb)
{
    ++frame_counter_;

    if (!playlist_.empty())
    {
        // 轮播实拍帧: 复制一份再叠加标注(不污染源图)
        rgb = playlist_[playlist_index_].clone();
        playlist_index_ = (playlist_index_ + 1) % playlist_.size();
    }
    else
    {
        // 程序合成: 渐变底 + 移动色块, 一眼可辨"这是模拟流"
        rgb.create(params_.sim_height, params_.sim_width, CV_8UC3);
        const int band = 60 + static_cast<int>(frame_counter_ % 40) * 2;
        for (int y = 0; y < rgb.rows; ++y)
        {
            const uint8_t v = static_cast<uint8_t>(
                60 + (y * 120 / std::max(rgb.rows, 1)));
            rgb.row(y).setTo(cv::Scalar(v, v, static_cast<uint8_t>(band)));
        }
        const int cx = static_cast<int>(
            (std::sin(frame_counter_ * 0.05) * 0.5 + 0.5) * (rgb.cols - 200));
        const int cy = rgb.rows / 2;
        cv::circle(rgb, cv::Point(cx, cy), 90, cv::Scalar(40, 160, 220), -1);
        cv::circle(rgb, cv::Point(rgb.cols - cx, cy), 60, cv::Scalar(220, 120, 40), -1);
    }

    // 叠加帧号/相机名/时间戳信息(验证画面"活着"与左右不串)
    const std::string text = cv::format(
        "%s frame#%llu t=%.1fs",
        params_.label.c_str(),
        static_cast<unsigned long long>(frame_counter_),
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count());
    cv::putText(rgb, text, cv::Point(16, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(rgb, text, cv::Point(16, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(80, 255, 80), 2, cv::LINE_AA);
}

GX_STATUS SimulatedGalaxyDevice::grab(
    GX_FRAME_DATA * frame_data, uint32_t timeout_ms)
{
    if (frame_data == nullptr || frame_data->pImgBuf == nullptr)
    {
        return GX_STATUS_INVALID_PARAMETER;
    }

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    // 按目标帧率节拍出图: 一直等到"下一帧时刻"或超时
    while (true)
    {
        std::chrono::steady_clock::time_point next;
        bool open = false;
        bool acquiring = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            open = open_;
            next = next_frame_time_;
            acquiring = acquiring_;
        }
        if (!open || !acquiring)
        {
            // 未打开/未开始采集: 与真机 GXGetImage 在同状态下返回
            // GX_STATUS_INVALID_CALL 一致, 由上层失败计数处理。
            return GX_STATUS_INVALID_CALL;
        }

        // 设备离线窗: 返回超时, 让节点走"连续超时判定掉线 -> 重连"
        if (!devicePresent())
        {
            return GX_STATUS_TIMEOUT;
        }

        if (std::chrono::steady_clock::now() >= next)
        {
            // 到点出图: 生成帧 -> 填 GX_FRAME_DATA
            cv::Mat rgb;
            produceFrame(rgb);

            const size_t payload =
                static_cast<size_t>(rgb.total()) * rgb.elemSize();
            std::memcpy(frame_data->pImgBuf, rgb.data, payload);

            frame_data->nWidth = rgb.cols;
            frame_data->nHeight = rgb.rows;
            frame_data->nFrameID = frame_counter_;
            frame_data->nImgSize = static_cast<int32_t>(payload);
            // nPixelFormat 置 0: 节点在 sim_mode 下不查 Bayer 排列,
            // 直接按 RGB24 打包(见 galaxy_camera_node.cpp 的 sim 分支)
            frame_data->nPixelFormat = 0;
            frame_data->nStatus = GX_FRAME_STATUS_SUCCESS;
            // nTimestamp: 微秒级 tick(1MHz), 与真机单位一致
            frame_data->nTimestamp = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - start_time_).count());

            std::lock_guard<std::mutex> lock(state_mutex_);
            const double period = 1.0 / params_.frame_rate;
            next_frame_time_ = std::chrono::steady_clock::now() +
                               std::chrono::duration_cast<
                                   std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(period));
            return GX_STATUS_SUCCESS;
        }

        if (std::chrono::steady_clock::now() >= deadline)
        {
            // 下一帧时刻晚于超时: 真机上低帧率时也会这样, 属正常超时
            return GX_STATUS_TIMEOUT;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

std::string SimulatedGalaxyDevice::describe()
{
    return "SIMULATED model=SIM-MER-500-14GC, name=" + name_ +
           ", playlist=" + std::to_string(playlist_.size()) + " frames";
}

}  // namespace galaxy_camera_dual
