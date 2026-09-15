// Copyright (c) 2024, culvert_core contributors
//
// 文件: core_node.hpp
// 作用: 涵洞检测核心节点声明(多源感知 + 同屏显示版)。
//
// 职责总览(话题精简: 只订阅 4 条数据话题 + 2 条辅助话题):
//   订阅:
//     /left_camera/image_raw/compressed   左大恒相机 JPEG
//     /right_camera/image_raw/compressed  右大恒相机 JPEG
//     /ir_camera/image_raw/compressed     红外机芯 JPEG(白热灰度)
//     /sensor/env                         温湿度/CO2(sensor_interfaces/EnvData)
//     /core_node/save_image               按键保存请求(辅助)
//   发布:
//     /core_node/status                   1Hz 心跳(帧率/传感器摘要, 辅助)
//
// 处理链(多线程, 各级解耦, 任何一路慢/断不拖累其他路):
//   [3 个图像回调(独立回调组, MultiThreadedExecutor 并发)]
//       -> JPEG 解码 -> 写入各自 StreamContext(仅保留最新帧)
//   [2 个 YOLO 检测线程: 左/右各一个独立 YoloDetector 实例]
//       cv::dnn::Net 非线程安全, 双实例并行推理, 避免互相排队
//       取最新帧 -> 缩放到窗格 -> 推理 -> 画框(红框+置信度+分割描边)
//   [1 个红外渗水检测线程]
//       IrSeepageDetector(背景差分) -> 黄色轮廓标注
//   [1 个显示线程]
//       三窗格同屏合成(canvas) + 传感器数据叠在总图左上角
//       -> imshow 单窗口 culvert_monitor; 窗格离线时显示占位提示
//
// 线程冲突处理约定(重要):
//   - StreamContext/PaneContext/环境数据/总图缓存 各有一把独立互斥锁;
//   - 任何函数同一时刻只持有一把锁, 不做嵌套加锁, 无死锁可能;
//   - 跨线程交接一律"锁内拷贝/swap + 锁外处理", 最重的工作(推理/GUI)
//     全部在锁外, 锁内只做小内存拷贝;
//   - 跨线程读写的标量(运行标志等)用 std::atomic。
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_interfaces/msg/env_data.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "culvert_core/ir_seepage_detector.hpp"
#include "culvert_core/yolo_detector.hpp"
#include "culvert_core/yolo_crack.hpp"

namespace culvert_core
{

class CoreNode : public rclcpp::Node
{
public:
    CoreNode();
    ~CoreNode() override;

private:
    // ==================== 视频源上下文(每路一份) ====================

    /// 一路视频源(左/右/红外)的最新帧缓存。
    /// 回调线程写入, 检测/保存线程读取, 由 mutex 保护;
    /// 只保留最新一帧: 消费慢时直接丢旧帧, 不堆积、不回压。
    struct StreamContext
    {
        std::string name;       ///< 参数/日志名: left / right / ir
        std::string title;      ///< 显示名(ASCII, imshow 不支持中文)
        std::string topic;      ///< 订阅的 compressed 话题

        std::mutex mutex;
        cv::Mat latest_bgr;     ///< 最新一帧(BGR, 已解码)
        rclcpp::Time latest_stamp{0, 0, RCL_ROS_TIME};  ///< 消息时间戳
        std::chrono::steady_clock::time_point latest_arrival{};  ///< 到达时刻(算离线)
        uint64_t seq = 0;       ///< 帧序号(每来一帧 +1, 检测线程排重用)

        // 帧率统计(PublishStatus 读, 回调线程写, 原子避免加锁)
        std::atomic<uint64_t> count{0};       ///< 累计收帧数
        std::atomic<uint64_t> count_prev{0};  ///< 上一秒累计(算 hz 用)


        /// 原图取帧率(帧间到达间隔的 EMA 平滑, 回调线程更新)。
        /// 供窗格信息条"实时显示当前取原相机原图的帧率"; 与检测输出
        /// 帧率(det, 检测线程 EMA)相互独立 —— 模型消费慢时 src 不变。
        std::atomic<double> src_fps{0.0};
    };

    /// 一个显示窗格的最新标注结果。
    /// 检测线程产出(锁内 swap), 显示线程消费(锁内拷贝);
    /// 推理/绘制慢时只影响画面刷新率, 不拖慢回调或其他检测线程。
    struct PaneContext
    {
        std::mutex mutex;
        cv::Mat annotated;      ///< 已画框的窗格图(窗格尺寸, BGR)
        uint64_t seq = 0;       ///< 对应的源帧序号
    };

    // ==================== 初始化 ====================

    /// 声明并读取全部 ROS 参数(话题/窗格/显示/YOLO/红外渗水)。
    void InitParams();

    /// 创建订阅/发布/定时器(图像订阅各自独立回调组, 支持并发解码)。
    void InitROS2();

    /// 创建 YOLO 检测器实例(左/右各一个, 独立加载模型)。
    void InitYolo();

    // ==================== 图像回调(JPEG 解码 -> 最新帧缓存) ====================

    /// 三路图像回调共用: JPEG 解码 -> 灰度转 BGR -> 写入 StreamContext。
    /// 解码在回调组线程中执行, 三路各自独立回调组 => 并发解码互不阻塞。
    void CompressedImageCallback(
        StreamContext & stream, const sensor_msgs::msg::CompressedImage & msg);

    /// 左/右/红外的订阅回调(薄封装, 保持日志可区分来源)。
    void LeftCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void RightCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void IrCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // ==================== 检测线程 ====================

    /// 可见光(左/右)YOLO 检测线程: 取最新帧 -> 缩放 -> 推理 -> 画框。
    /// detector 为该线程独占(两路各一个实例, 避免共享推理队列);
    /// detector 为 nullptr 时纯透传(不推理, 仅缩放), 用于 yolo 关闭时
    /// 保证该窗格仍有画面。
    void VisibleDetectLoop(
        StreamContext & stream, PaneContext & pane,
        std::unique_ptr<YoloDetector> & detector,
        const std::string & window_tag);

    /// 红外渗水检测线程: 取最新帧 -> 缩放 -> 背景差分 -> 画渗水轮廓。
    void IrDetectLoop();

    /// 从 StreamContext 取最新帧(锁内拷贝, 与 seq 排重: 无新帧返回 false)。
    bool FetchLatest(StreamContext & stream, uint64_t & last_seq, cv::Mat & frame);

    /// 把标注好的窗格交给显示线程(锁内 swap, 只保留最新)。
    void SubmitPane(PaneContext & pane, cv::Mat & annotated, uint64_t seq);

    // ==================== 同屏显示线程 ====================

    /// 显示线程: 三窗格合成 + 传感器左上角标注 + 占位提示 + imshow。
    void DisplayLoop();

    /// 传感器数据标注块(画在总图左上角, 半透明底 + 两行 ASCII 文本)。
    void DrawSensorOverlay(cv::Mat & canvas);
    /// 导航协议标注块: 位于第二窗格顶部，显示收到的指令/发出的状态/状态词。
    void DrawNavOverlay(cv::Mat & canvas);

    /// 窗格顶部信息条(标题 + 帧率 + 检测摘要)。
    void DrawPaneHeader(cv::Mat & pane_img, const std::string & title,
                        const std::string & info);

    // ==================== 传感器回调 ====================

    /// 环境数据回调: 存最新值(锁内拷贝), 供显示/状态发布读取。
    void EnvCallback(const sensor_interfaces::msg::EnvData::SharedPtr msg);

    // ==================== 状态/保存 ====================

    /// 1Hz 心跳: 三路帧率 + 传感器摘要(话题 + 节流日志双输出)。
    void PublishStatus();

    /// 保存当前一帧: 总图(含标注/传感器) + 三路最新帧, 存 JPEG。
    void SaveCurrentFrame();

    /// cv::Mat 存 JPEG 到 save_dir_(文件名带时间与 tag), 返回是否成功。
    bool WriteMatToJpeg(const std::string & tag, const cv::Mat & image);

    /// 按 S 键保存(仅本进程 stdin 是终端时启用, 如 ros2 run 直跑)。
    void KeyListenLoop();

    // ==================== ROS2 通信对象 ====================

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr ir_sub_;
    rclcpp::Subscription<sensor_interfaces::msg::EnvData>::SharedPtr env_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_req_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 图像订阅的独立回调组(三路解码并发, 互不阻塞)
    rclcpp::CallbackGroup::SharedPtr left_cb_group_;
    rclcpp::CallbackGroup::SharedPtr right_cb_group_;
    rclcpp::CallbackGroup::SharedPtr ir_cb_group_;

    // ==================== 参数 ====================

    std::string save_dir_;             ///< 按 S 保存目录
    bool use_sensor_data_qos_ = true;  ///< 订阅 QoS(与驱动发布端一致)
    int pane_width_ = 640;             ///< 单窗格宽(像素)
    int pane_height_ = 480;            ///< 单窗格高(像素)
    int pane_gap_ = 4;                 ///< 窗格间隔(像素)
    double display_fps_ = 30.0;        ///< 显示刷新率上限(Hz)
    double stale_timeout_sec_ = 3.0;   ///< 超过该时长无新帧判定该路离线
    bool show_img_ = true;             ///< 是否弹出同屏显示窗口

    // ---- 导航协议可视化(仅订阅展示, 不干预协议) ----
    std::string vision_cmd_topic_ = "/vision_capture_cmd";
    std::string vision_status_topic_ = "/vision_capture_status";
    std::atomic<uint8_t> nav_cmd_{0};      ///< 最近收到的导航指令字节
    std::atomic<uint8_t> nav_status_{0};   ///< 最近发出的状态字节(订阅同话题)
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_status_sub_;

    // 导航协议新鲜度(面板红色告警; 双话题回调与显示线程并发, 互斥保护)。
    // 电平协议本身不带"是否在线"信息: 导航不发指令时面板看到的仍是上次
    // 电平(恰好行车时是 0x00), 无法区分"在行车"与"导航没接入", 故用
    // 到达时刻做新鲜度校验 —— 超阈即红框告警, 恢复后自动复原。
    std::mutex nav_mutex_;                     ///< 保护以下到达时刻/取图周期
    std::chrono::steady_clock::time_point nav_cmd_recv_;    ///< 最近收到导航指令
    bool has_nav_cmd_ = false;                 ///< 是否收到过导航指令
    std::chrono::steady_clock::time_point nav_status_recv_; ///< 最近收到视觉状态
    bool has_nav_status_ = false;              ///< 是否收到过视觉状态
    std::chrono::steady_clock::time_point capture_since_;   ///< 本周期首次 0x01 时刻
    bool capturing_ = false;                   ///< 取图周期进行中(0x01 起 0x00/0x02 止)
    double nav_cmd_fresh_sec_ = 2.0;           ///< 指令新鲜度阈值(超则红框, json 可调)
    double capture_done_timeout_sec_ = 2.0;    ///< 0x01 后取图完成时限(超则红框, json 可调)

    // ---- 标注图发布(巡检保存"模型处理完框出来的图"用) ----
    bool publish_annotated_ = true;   ///< 是否发布标注图话题
    int annotated_quality_ = 85;      ///< 标注图 JPEG 质量
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
        left_annotated_pub_;   ///< /core_node/left_annotated
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
        right_annotated_pub_;  ///< /core_node/right_annotated
    bool fullscreen_ = true;           ///< 全屏展示(无边框占满整屏; 不用 GTK 独占全屏以免闪黑)
    int screen_w_ = 1920;              ///< 屏幕宽(0=启动时 xrandr 自动探测)
    int screen_h_ = 1080;              ///< 屏幕高(0=启动时 xrandr 自动探测)

    // ---- YOLO 参数(左右共用模型配置, 各自独立实例) ----
    bool yolo_enable_left_ = true;     ///< 左相机是否跑 YOLO
    bool yolo_enable_right_ = true;    ///< 右相机是否跑 YOLO
    std::string yolo_model_path_;      ///< ONNX 模型路径
    std::string yolo_model_type_;      ///< "seg"(框+描边) / "detect"(仅框)
    bool yolo_use_gpu_ = false;        ///< OpenVINO GPU(默认 CPU)
    float yolo_conf_threshold_ = 0.35f;
    float yolo_nms_threshold_ = 0.45f;
    int yolo_input_size_ = 640;
    int yolo_max_threads_ = 8;         ///< OpenCV 线程池上限(留核给解码/显示)

    // ---- 红外渗水检测参数(与 IrSeepageParams 对应) ----
    bool ir_enable_ = true;            ///< 是否跑渗水检测(false 时纯透传)
    IrSeepageParams ir_params_{};      ///< 渗水算法参数(ksize/阈值/形态学等)

    // ==================== 三路视频源与窗格 ====================

    StreamContext left_{};             ///< 左大恒
    StreamContext right_{};            ///< 右大恒
    StreamContext ir_{};               ///< 红外机芯
    PaneContext pane_left_{};          ///< 左窗格标注结果
    PaneContext pane_right_{};         ///< 右窗格标注结果
    PaneContext pane_ir_{};            ///< 红外窗格标注结果

    // ==================== 环境数据(锁内更新) ====================

    std::mutex env_mutex_;
    sensor_interfaces::msg::EnvData latest_env_;   ///< 最新环境数据
    bool has_env_ = false;                          ///< 是否收到过
    std::chrono::steady_clock::time_point env_arrival_{};  ///< 到达时刻(算数据年龄)

    // ==================== 线程与运行标志 ====================

    std::unique_ptr<YoloDetector> yolo_left_;   ///< 左 YOLO(左检测线程独占)
    std::unique_ptr<YoloDetector> yolo_right_;  ///< 右 YOLO(右检测线程独占)
    std::unique_ptr<IrSeepageDetector> ir_detector_;  ///< 红外渗水检测器

    std::thread left_detect_thread_;
    std::thread right_detect_thread_;
    std::thread ir_detect_thread_;
    std::thread display_thread_;
    std::thread key_thread_;

    std::atomic<bool> detect_run_{false};   ///< 检测线程运行标志
    std::atomic<bool> display_run_{false};  ///< 显示线程运行标志
    std::atomic<bool> key_run_{false};      ///< 按键线程运行标志

    // ==================== 总图缓存(显示线程写/保存线程读) ====================

    std::mutex composite_mutex_;
    cv::Mat last_composite_;       ///< 最近一次合成的总图(含全部标注)
};

}  // namespace culvert_core
