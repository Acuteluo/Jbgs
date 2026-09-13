// Copyright (c) 2024, galaxy_camera_dual contributors
//
// 文件: simulated_galaxy_device.hpp
// 作用: 大恒相机的"模拟设备"声明 —— 无实体相机时, 用它顶替 GalaxyDevice。
//
// 设计目标(为什么放在 SDK 封装层, 而不是在节点里到处写 if(sim)):
//   热插拔测试要测的是**节点自身的恢复逻辑**(超时计数 -> 停采 -> 关设备 ->
//   重开 -> 恢复参数 -> 重采), 因此模拟设备必须和真机走同一条代码路径。
//   做法: 继承 GalaxyDevice, 覆写全部设备操作; 节点持有的仍是基类指针,
//   对它来说"模拟设备"与"真机"不可区分。
//
// 模拟的插拔语义(与实体 GigE 相机的掉线/上线一一对应):
//   - 设备在位 = 当前时刻不处于"离线时间窗";
//   - 初始离线窗  [0, initial_absent_sec)              -> 对应"启动时未接入";
//   - 中途离线窗  从 disconnect_at_sec 开始, 每次持续
//     disconnect_duration_sec; 若 disconnect_repeat_period_sec > 0
//     则每隔该周期重复一次(模拟频繁插拔的极端情况)     -> 对应"中途拔掉";
//   - 离线窗内: open() 失败(等同 GXOpenDevice 找不到设备);
//     grab() 返回 GX_STATUS_TIMEOUT(等同设备掉线后收不到帧),
//     由节点既有的"连续超时判定掉线 -> 重连"逻辑自动恢复。
//
// 帧内容:
//   - 优先轮播 image_dir 下的 jpg(如按 S 保存的实拍帧, 上面有真实裂缝,
//     可让下游 YOLO 真的检出框, 便于验证整条标注链路);
//   - 目录为空时退化为程序生成的动态测试图(移动色块 + 帧号 + 相机名),
//     保证画面"活着"一眼可辨。
// 输出像素: 直接生成 3 通道 RGB(跳过 Bayer), 节点采集循环里有对应的
//   sim_mode 分支直接按 rgb8 打包, 不走 DxRaw8toRGB24。

#pragma once

#include "galaxy_camera_dual/galaxy_device.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace galaxy_camera_dual
{

/// 模拟设备的插拔/出图参数(由节点从 ROS 参数填充)。
struct SimDeviceParams
{
    double initial_absent_sec = 8.0;   ///< 启动后设备离线时长(秒), 0 = 开机即在位
    double disconnect_at_sec = 30.0;   ///< 首次"中途拔掉"的时刻(秒), <=0 = 不模拟中途断连
    double disconnect_duration_sec = 8.0;  ///< 每次中途离线的持续时间(秒)
    double disconnect_repeat_period_sec = 0.0;  ///< >0 时每隔该秒数重复一次离线窗
    double frame_rate = 5.0;           ///< 模拟出图帧率(Hz); <=0 时用 14fps(手册标称)
    int sim_width = 1280;              ///< 模拟帧宽(像素)
    int sim_height = 960;              ///< 模拟帧高(像素)
    std::string image_dir;             ///< 轮播的实拍帧目录(空 = 程序合成图)
    std::string label = "SIM";         ///< 画面上标注的相机名(便于区分左右)
};

/// 模拟大恒相机: 与 GalaxyDevice 同一接口, 可无缝替换。
class SimulatedGalaxyDevice : public GalaxyDevice
{
public:
    /// name 仅用于日志与画面标注; params 为插拔/出图配置。
    SimulatedGalaxyDevice(const std::string & name, const SimDeviceParams & params);
    ~SimulatedGalaxyDevice() override;

    // ---- 打开/关闭/状态(与 GalaxyDevice 语义一致) ----
    bool open(const DeviceAddress & address, std::string * error_msg = nullptr) override;
    void close() override;
    bool isOpen() const override;

    // ---- 特性读写: 绝大多数为"接受并返回成功"的空操作, 少数返回
    //      与真机一致的关键值(载荷大小 / tick 频率), 让节点的时间戳、
    //      缓冲分配等逻辑原样运行。 ----
    bool getInt(GX_FEATURE_ID_CMD id, int64_t * value) override;
    bool setInt(GX_FEATURE_ID_CMD id, int64_t value) override;
    bool getFloat(GX_FEATURE_ID_CMD id, double * value) override;
    bool setFloat(GX_FEATURE_ID_CMD id, double value) override;
    bool setBool(GX_FEATURE_ID_CMD id, bool value) override;
    bool sendCommand(GX_FEATURE_ID_CMD id) override;
    bool getFloatRange(GX_FEATURE_ID_CMD id, GX_FLOAT_RANGE * range) override;
    bool setEnum(GX_FEATURE_ID_CMD id, int64_t value) override;
    bool getString(GX_FEATURE_ID_CMD id, std::string * value) override;
    bool setString(GX_FEATURE_ID_CMD id, const std::string & value) override;
    bool isImplemented(GX_FEATURE_ID_CMD id) override;
    bool setAcquisitionBufferNumber(uint64_t buffer_count) override;
    bool startAcquisition() override;
    bool stopAcquisition() override;
    bool flushQueue() override;

    /// 阻塞式取一帧: 按目标帧率生成; 离线窗内返回 GX_STATUS_TIMEOUT,
    /// 触发节点既有的掉线判定与自动重连。
    GX_STATUS grab(GX_FRAME_DATA * frame_data, uint32_t timeout_ms) override;

    std::string describe() override;

private:
    /// 当前时刻设备是否在位(不在离线时间窗内)。
    bool devicePresent() const;
    /// 当前时刻是否处于离线时间窗(供 open/grab 共用)。
    bool inAbsentWindow(std::chrono::steady_clock::time_point t) const;
    /// 生成一帧 RGB 测试图(轮播实拍帧或程序合成)。
    void produceFrame(cv::Mat & rgb);

    std::string name_;              ///< 相机组名(left/right), 用于日志
    SimDeviceParams params_;        ///< 插拔/出图参数
    std::chrono::steady_clock::time_point start_time_;  ///< 设备对象创建时刻(时间窗基准)

    // mutable: const 方法(isOpen/devicePresent/inAbsentWindow)也要加锁
    mutable std::mutex state_mutex_;  ///< 保护 open_/acquiring_/next_frame_time_
    bool open_ = false;             ///< 是否处于"已打开"状态
    bool acquiring_ = false;        ///< 是否已开始采集(start/stop 命令切换)
    std::chrono::steady_clock::time_point next_frame_time_{};  ///< 下一帧应出图的时刻

    cv::Mat frame_buffer_;          ///< 最新生成的模拟帧(RGB, 复用以减少分配)
    std::vector<cv::Mat> playlist_; ///< 预加载的实拍轮播帧(缩放后)
    size_t playlist_index_ = 0;     ///< 轮播游标
    uint64_t frame_counter_ = 0;    ///< 已生成帧数(画面上显示, 也作时间戳用)
    bool playlist_ready_ = false;   ///< 实拍帧是否已加载(懒加载, 首次 open 后执行)
};

}  // namespace galaxy_camera_dual
