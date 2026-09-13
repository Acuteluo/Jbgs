// Copyright (c) 2024, ir_camera_driver contributors
//
// 文件: ir_camera_node.hpp
// 作用: 红外热成像(UVC/V4L2)相机驱动节点声明。
//
// 背景(设备与协议):
//  - 涵洞巡检机器人搭载的红外机芯(MIPI 模组经转接板)以标准 UVC 设备
//    出流, 输出白热(White-Hot)伪彩/灰度图, 无需厂家测温 SDK;
//  - 本节点用 OpenCV VideoCapture(CAP_V4L2) 打开 /dev/videoN 取流,
//    与 Culvert-Visual-Inspection-main 的接入方式一致;
//  - 渗水检测(IrSeepageDetector, 背景差分)不在本节点做 —— 本节点只做
//    "驱动": 采集 + JPEG 发布, 保持与大恒相机驱动同样的极简职责,
//    检测/画框/同屏合成统一在 culvert_core 的 core_node 完成。
//
// 话题(精简约定: 本节点只发布一条话题):
//    <topic_base>/compressed  (sensor_msgs/CompressedImage, JPEG)
//    topic_base 默认 /ir_camera/image_raw -> 实际话题
//    /ir_camera/image_raw/compressed
//
// 线程模型(多线程要点, 与 galaxy_camera_dual 同风格):
//  - 独立采集线程 captureThread: 阻塞式取流 + JPEG 编码 + 发布,
//    取流慢/失败绝不会拖住 ROS 执行器线程;
//  - ROS 执行器线程: 只处理参数服务, 不碰相机对象;
//  - 关闭顺序: running_=false -> 释放 VideoCapture(唤醒阻塞读) -> join,
//    全程无死锁、无残留线程;
//  - 热插拔自愈: 连续 grab_fail_limit_ 次失败(含设备被拔时的 read 失败)
//    判定离线 -> 释放设备 -> 采集线程内每 1 秒重试重开, 恢复后自动续流;
//    全过程不退出线程、不影响 ROS 执行器与其他节点。
#pragma once

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/videoio.hpp>

namespace ir_camera_driver
{

class IrCameraNode : public rclcpp::Node
{
public:
    explicit IrCameraNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~IrCameraNode() override;

private:
    // ==================== 初始化 ====================

    /// 声明并读取全部参数(话题名/设备号/帧率/模拟参数等)。
    void initParams();

    /// 创建发布器并启动采集线程。
    void initRos2AndStart();

    // ==================== 采集线程 ====================

    /// 采集主循环: 打开设备 -> 取帧 -> 灰度化 -> 限帧 -> JPEG 发布;
    /// 连续失败进入热插拔恢复(关闭 + 重试), 设备重新接入后自动续流。
    /// 循环体外层做异常安全护栏(任何异常详细记录后继续, 不退出线程)。
    void captureThread();

    /// 采集单次处理(被 captureThread 的异常护栏包裹):
    /// 健康报告 / 离线重试打开 / 取帧 / 限帧 / 编码发布。
    void captureOnce(std::vector<unsigned char> & jpeg_buffer);

    /// 尝试打开设备(真机: VideoCapture; 模拟: 检查在位时间窗)。
    /// 返回是否成功; 成功时 opened_=true。
    bool tryOpenDevice();

    /// 取一帧灰度图(真机: cap.read 后转灰度; 模拟: 生成合成热像)。
    /// 返回 false 表示本次取帧失败(超时/坏帧/设备消失)。
    bool grabGrayFrame(cv::Mat & gray);

    /// 关闭设备(真机: release; 模拟: 复位标志)。幂等。
    void closeDevice();

    /// 模拟模式: 当前时刻设备是否在位(离线时间窗外)。
    bool simDevicePresent() const;

    /// 模拟模式: 生成一帧 8bit 白热灰度图(渐变底 + 噪声 + 移动冷斑),
    /// 冷斑即"模拟渗水", 供下游 IrSeepageDetector 真实检出。
    void produceSimFrame(cv::Mat & gray);

    // ==================== 参数回调 ====================

    /// 动态参数: 在线改 jpeg_quality; 其余只读(需重启)。
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> & params);

    // ==================== ROS2 通信对象 ====================

    /// JPEG 发布器, 话题 <topic_base>/compressed(本节点唯一话题)。
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ==================== 参数(只读, 构造时确定) ====================

    std::string topic_base_ = "/ir_camera/image_raw";  ///< 话题基名
    int device_index_ = 0;         ///< 真机: V4L2 设备序号(/dev/videoN 的 N)
    std::string device_path_;      ///< 真机: 设备路径(非空时优先于序号)
    double frame_rate_ = 25.0;     ///< 发布帧率上限(Hz); <=0 = 跟随设备
    int grab_fail_limit_ = 10;     ///< 连续失败多少次判定离线(触发热插拔恢复)
    bool use_sensor_data_qos_ = true;  ///< 发布 QoS(与订阅端 core_node 对齐)
    std::string frame_id_ = "ir_camera_optical_frame";  ///< 图像 frame_id

    // ---- 模拟模式参数(插拔语义与 galaxy_camera_dual 完全一致) ----
    bool sim_mode_ = true;             ///< true = 无实体机芯, 用模拟热像
    double sim_initial_absent_sec_ = 8.0;   ///< 启动后模拟"未接入"时长
    double sim_disconnect_at_sec_ = 35.0;   ///< 首次"中途拔掉"时刻; <=0 关闭
    double sim_disconnect_duration_sec_ = 8.0;  ///< 每次中途离线时长
    double sim_disconnect_repeat_period_sec_ = 0.0;  ///< >0 = 周期重复断连
    double sim_fps_ = 25.0;            ///< 模拟出图帧率
    int sim_width_ = 256;              ///< 模拟帧宽(机芯典型 256x192)
    int sim_height_ = 192;             ///< 模拟帧高

    // ==================== 采集线程状态 ====================

    std::atomic<bool> running_{false};   ///< 线程运行标志(析构置 false)
    std::thread thread_;                 ///< 采集线程(唯一自建线程)
    bool opened_ = false;                ///< 设备当前是否已打开(仅采集线程读写)
    cv::VideoCapture cap_;               ///< 真机取流对象(仅采集线程使用)
    int fail_count_ = 0;                 ///< 连续取帧失败计数

    // ---- 诊断统计(仅采集线程读写, 周期打印健康报告) ----
    uint64_t stat_ok_ = 0;         ///< 完整帧计数
    uint64_t stat_fail_ = 0;       ///< 失败/离线计数
    uint64_t stat_reopen_ = 0;     ///< 热插拔重开次数
    std::chrono::steady_clock::time_point next_report_{};  ///< 下次健康报告时刻
    std::chrono::steady_clock::time_point start_time_{};   ///< 启动时刻(模拟时间窗基准)

    // ---- 限帧与帧内容状态(仅采集线程使用) ----
    std::chrono::steady_clock::time_point next_frame_time_{};    ///< 模拟出图节拍(下一帧时刻)
    std::chrono::steady_clock::time_point next_publish_time_{};  ///< 发布限帧节拍(与出图节拍独立, 避免双重推进互相踩踏)
    uint64_t frame_counter_ = 0;   ///< 已发布帧号(叠加在模拟画面上)
    // JPEG 质量: 参数回调线程可在线修改、采集线程读取 —— 跨线程,
    // 必须用原子类型(普通 int 的并发读写是数据竞争)。
    std::atomic<int> jpeg_quality_{80};  ///< JPEG 质量(1-100)
};

}  // namespace ir_camera_driver
