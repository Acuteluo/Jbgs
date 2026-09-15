// 文件: inspection_node.hpp
// 作用: 涵洞巡检通信协议节点声明(rclcpp 薄封装)。
//
// 协议逻辑(状态机)全部在 inspection_fsm.hpp 的 InspectionFsm,
// 本节点只负责 ROS2 侧: 参数装载、话题订阅/发布、回调组线程模型、
// 日志桥接、文件写入与 OpenCV 解码的生产实现注入。
//
// 线程模型(MultiThreadedExecutor + 独立回调组):
//   - 左/右图像回调、指令回调、看门狗定时器并发进入;
//   - 图像"空闲快路径"仅原子到达计数(无锁、零拷贝);
//   - Armed 之后协议处理由 state_mutex_ 串行化(锁内不拷贝大对象,
//     仅引用原始 JPEG 字节);
//   - 指令回调用 try_lock: 锁被占用 = 正在写盘 => "处理中再次收到
//     0x01", 节流告警并忽略, 不排队成第二次巡检、绝不误发 0x02。
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "culvert_inspection/inspection_fsm.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace culvert_inspection
{

class InspectionNode : public rclcpp::Node
{
public:
    InspectionNode();
    ~InspectionNode() override;

    /// 状态名(状态话题/日志用): IDLE/ARMED/SAVING。
    static const char * StateName(int state);

private:
    // ---- 参数 ----
    bool in_trulyworking_ = false;       ///< 巡检模式总开关
    std::string input_topic_;            ///< 导航->视觉指令话题(UInt8)
    std::string ack_topic_;              ///< 视觉->导航状态话题(UInt8)
    double status_rate_hz_ = 20.0;       ///< 协议状态发布频率(可调)
    double annot_gap_ms_ = 500.0;        ///< 标注图与原图时间戳配对容差
    std::string left_annotated_topic_;   ///< 左标注图话题(culvert_core 发布)
    std::string right_annotated_topic_;  ///< 右标注图话题
    bool use_sensor_data_qos_ = true;    ///< 与驱动发布端 QoS 一致

    // ---- 回调 ----
    /// 巡检指令回调(UInt8): try_lock 串行化, 处理中再收 0x01 忽略。
    void CommandCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    /// 图像回调: 空闲快路径计数; Armed 后锁内交状态机选择目标帧。
    void ImageCallback(
        InspectionFsm::Side & side,
        sensor_msgs::msg::CompressedImage::SharedPtr msg);
    /// 看门狗(10Hz): 无进展超时检查。
    void WatchdogTimer();
    /// 协议状态定时发布(可调频率): 0x00 空闲 / 0x01 保存中 / 0x02 完成。
    /// 电平语义: 持续广播当前状态, 不是一次性确认 —— 0x02 会一直保持到
    /// 收到导航 0x00(周期结束) 才回到 0x00。
    void StatusTimer();
    /// 标注图回调: Armed 且原图已捕获时, 按时间戳容差配对暂存。
    void AnnotatedCallback(
        InspectionFsm::Side & side,
        sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // ---- ROS2 对象 ----
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;       ///< 导航协议状态
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_str_pub_;  ///< 调试状态(String)
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        left_ann_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        right_ann_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr right_sub_;
    rclcpp::CallbackGroup::SharedPtr left_cb_group_;   ///< 左解码独立回调组
    rclcpp::CallbackGroup::SharedPtr right_cb_group_;  ///< 右解码独立回调组
    rclcpp::CallbackGroup::SharedPtr cmd_cb_group_;    ///< 指令独立回调组

    // ---- 状态机与串行化 ----
    std::unique_ptr<InspectionFsm> fsm_;
    std::mutex state_mutex_;       ///< 协议处理串行化(Armed/Saving 期间)
    bool done_await_zero_ = false; ///< 完成后等导航回 0x00(防残留 0x01 误触发)
    std::atomic<uint8_t> last_cmd_level_{0};  ///< 导航指令电平(0x00/0x01)
};

}  // namespace culvert_inspection
