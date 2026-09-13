// Copyright (c) 2024, ir_camera_driver contributors
//
// 文件: ir_camera_node.cpp
// 作用: 红外热成像(UVC/V4L2)相机驱动节点实现。
//
// 与 galaxy_camera_dual 的对齐关系(合并成一套系统的关键):
//  - 同样"每设备一线程"采集 + 独立热插拔自愈, 任一设备离线不影响他人;
//  - 同样只发布 JPEG 压缩话题(话题精简: 本节点全生命周期只有一条话题);
//  - 模拟模式的插拔时间窗参数与 galaxy 相机同构
//    (initial_absent / disconnect_at / duration / repeat_period),
//    便于用同一套测试脚本驱动全部设备的"初始断连/中途断连"场景。
//
// 热插拔状态机(采集线程内, 无锁切换):
//   [离线] --tryOpenDevice成功--> [在线]
//   [在线] --连续 grab_fail_limit_ 次取帧失败--> closeDevice --> [离线]
//   [离线] --每 1s 重试--> 设备重新出现后回到 [在线]
// 真机被拔线时 UVC read 返回空帧/失败, 自然落入"连续失败"路径;
// 模拟模式则由离线时间窗直接产生同样的失败序列 —— 两者走完全相同的
// 恢复代码, 保证模拟测试对真机行为有代表性。

#include "ir_camera_driver/ir_camera_node.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace ir_camera_driver
{

namespace
{
/// 离线状态下重试打开设备的间隔(秒)。
constexpr double kReopenIntervalSec = 1.0;

/// "等待接入"日志的节流间隔(毫秒)。
constexpr int kWaitingLogThrottleMs = 15000;

/// 健康报告间隔(秒)。
constexpr double kHealthReportSec = 30.0;
}  // namespace

// ============================== 构造/析构 ==============================

IrCameraNode::IrCameraNode(const rclcpp::NodeOptions & options)
: Node("ir_camera_driver", options)
{
    RCLCPP_INFO(get_logger(), "IrCameraNode 启动");

    initParams();
    initRos2AndStart();
}

IrCameraNode::~IrCameraNode()
{
    RCLCPP_INFO(get_logger(), "IrCameraNode 关闭中...");

    // 关闭顺序(不能调换):
    //  1) 置 running_=false, 让采集线程退出主循环;
    //  2) join 线程: 必须先等线程退出再动设备 —— VideoCapture 非线程安全,
    //     主线程并发 release() 与采集线程的 read() 是数据竞争;
    //     UVC 流帧持续到达或失败返回, read 不会永久阻塞, join 可达;
    //  3) 释放设备(线程已退出, 独占访问安全)。
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
    closeDevice();
    RCLCPP_INFO(get_logger(), "IrCameraNode 已退出");
}

// ============================== 参数 ==============================

void IrCameraNode::initParams()
{
    topic_base_ = declare_parameter<std::string>("topic_base", "/ir_camera/image_raw");
    device_index_ = declare_parameter<int>("device_index", 0);
    device_path_ = declare_parameter<std::string>("device_path", "");
    frame_rate_ = declare_parameter<double>("frame_rate", 25.0);
    grab_fail_limit_ = declare_parameter<int>("grab_fail_limit", 10);
    use_sensor_data_qos_ = declare_parameter<bool>("use_sensor_data_qos", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "ir_camera_optical_frame");
    jpeg_quality_ = static_cast<int>(declare_parameter<int>("jpeg_quality", 80));

    // ---- 模拟模式参数(与 galaxy_camera_dual 同构) ----
    sim_mode_ = declare_parameter<bool>("sim_mode", true);
    sim_initial_absent_sec_ = declare_parameter<double>("sim.initial_absent_sec", 8.0);
    sim_disconnect_at_sec_ = declare_parameter<double>("sim.disconnect_at_sec", 35.0);
    sim_disconnect_duration_sec_ =
        declare_parameter<double>("sim.disconnect_duration_sec", 8.0);
    sim_disconnect_repeat_period_sec_ =
        declare_parameter<double>("sim.disconnect_repeat_period_sec", 0.0);
    sim_fps_ = declare_parameter<double>("sim.fps", 25.0);
    sim_width_ = declare_parameter<int>("sim.width", 256);
    sim_height_ = declare_parameter<int>("sim.height", 192);

    RCLCPP_INFO(
        get_logger(),
        "参数: topic=%s%s, %s, frame_rate=%.1f, fail_limit=%d%s",
        topic_base_.c_str(), "/compressed",
        sim_mode_ ? "模拟模式" : "真机(UVC)",
        frame_rate_, grab_fail_limit_,
        sim_mode_ ?
            cv::format("(初始离线 %.1fs, 中途断连 %.1fs起/%.1fs长, 重复 %.1fs)",
                       sim_initial_absent_sec_, sim_disconnect_at_sec_,
                       sim_disconnect_duration_sec_,
                       sim_disconnect_repeat_period_sec_).c_str() : "");
}

void IrCameraNode::initRos2AndStart()
{
    const rclcpp::QoS qos = use_sensor_data_qos_ ?
        static_cast<rclcpp::QoS>(rclcpp::SensorDataQoS()) :
        rclcpp::SystemDefaultsQoS();

    // 唯一话题: <topic_base>/compressed(遵循 image_transport 命名惯例)
    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_base_ + "/compressed", qos);

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params)
        {
            return onParameterChange(params);
        });

    // 启动采集线程(设备未就绪也照常启动: 线程内部会做热插拔等待)
    start_time_ = std::chrono::steady_clock::now();
    next_report_ = start_time_ + std::chrono::seconds(
        static_cast<int64_t>(kHealthReportSec));
    running_ = true;
    thread_ = std::thread(&IrCameraNode::captureThread, this);
}

rcl_interfaces::msg::SetParametersResult IrCameraNode::onParameterChange(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params)
    {
        if (p.get_name() == "jpeg_quality" &&
            p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            // 原子写入, 采集线程下一帧即生效
            jpeg_quality_ =
                std::clamp(static_cast<int>(p.as_int()), 1, 100);
        }
        else
        {
            result.successful = false;
            result.reason = "参数 '" + p.get_name() + "' 只读, 需重启";
        }
    }
    return result;
}

// ============================== 热插拔状态机 ==============================

bool IrCameraNode::simDevicePresent() const
{
    const double elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

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

bool IrCameraNode::tryOpenDevice()
{
    if (sim_mode_)
    {
    // 模拟: 在位 = 打开成功; 离线 = 与真机"设备不存在"一致
    if (!simDevicePresent())
    {
        return false;
    }
    frame_counter_ = 0;
    next_frame_time_ = std::chrono::steady_clock::now();
    next_publish_time_ = std::chrono::steady_clock::now();
    return true;
    }

    // 真机: 优先用路径(如 /dev/video2), 否则用 V4L2 序号
    cap_.release();
    const bool ok = !device_path_.empty() ?
        cap_.open(device_path_, cv::CAP_V4L2) :
        cap_.open(device_index_, cv::CAP_V4L2);
    if (!ok)
    {
        return false;
    }
    // MJPG 请求: 多数 UVC 机芯在 MJPG 下才能跑满帧率; 失败也无妨
    // (机芯不支持时 OpenCV 保持默认 YUYV, 帧率可能受限)。
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    frame_counter_ = 0;
    next_frame_time_ = std::chrono::steady_clock::now();
    next_publish_time_ = std::chrono::steady_clock::now();
    return true;
}

void IrCameraNode::closeDevice()
{
    if (cap_.isOpened())
    {
        cap_.release();
    }
    opened_ = false;
}

bool IrCameraNode::grabGrayFrame(cv::Mat & gray)
{
    if (sim_mode_)
    {
        // 模拟取帧: 严格按 sim_fps 节拍出图(与真机帧节奏一致);
        // 出图节拍(next_frame_time_)与发布限帧(next_publish_time_)相互独立,
        // 各自推进, 避免同一时间轴被双重累加导致节奏漂移。
        const double period = (sim_fps_ > 0.0) ? (1.0 / sim_fps_) : 0.04;
        std::this_thread::sleep_until(next_frame_time_);
        next_frame_time_ =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(period));
        // 离线窗内返回失败, 触发与真机拔线相同的恢复路径
        if (!simDevicePresent())
        {
            return false;
        }
        produceSimFrame(gray);
        return true;
    }

    // 真机取帧: 阻塞式 read(机芯内部缓冲; 被拔线时返回空帧)
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty())
    {
        return false;
    }
    if (frame.channels() == 1)
    {
        gray = frame;
    }
    else if (frame.channels() == 3)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else if (frame.channels() == 4)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    }
    else
    {
        frame.convertTo(gray, CV_8UC1);
    }
    return true;
}

// ============================== 模拟热像生成 ==============================

void IrCameraNode::produceSimFrame(cv::Mat & gray)
{
    ++frame_counter_;

    // 1) 基础场: 垂直渐变(模拟墙面温度梯度) + 高斯噪声(探测器 NETD)
    gray.create(sim_height_, sim_width_, CV_8UC1);
    for (int y = 0; y < sim_height_; ++y)
    {
        const uint8_t v = static_cast<uint8_t>(110 + y * 40 / std::max(sim_height_, 1));
        gray.row(y).setTo(v);
    }
    cv::Mat noise(sim_height_, sim_width_, CV_8SC1);
    cv::randu(noise, -6, 6);
    gray += noise;

    // 2) 移动冷斑 x2: 模拟"渗水区域"(白热图上偏暗 -> Diff 为正可检出)
    auto add_blob = [&gray](cv::Point2f center, float sigma, int16_t amplitude)
    {
        const int r = static_cast<int>(sigma * 3.0f);
        const int x0 = std::max(0, static_cast<int>(center.x) - r);
        const int y0 = std::max(0, static_cast<int>(center.y) - r);
        const int x1 = std::min(gray.cols, static_cast<int>(center.x) + r);
        const int y1 = std::min(gray.rows, static_cast<int>(center.y) + r);
        for (int y = y0; y < y1; ++y)
        {
            for (int x = x0; x < x1; ++x)
            {
                const float dx = x - center.x;
                const float dy = y - center.y;
                const float g = std::exp(
                    -(dx * dx + dy * dy) / (2.0f * sigma * sigma));
                int v = gray.at<uint8_t>(y, x) +
                        static_cast<int>(amplitude * g);
                gray.at<uint8_t>(y, x) =
                    static_cast<uint8_t>(std::clamp(v, 0, 255));
            }
        }
    };

    // 两个冷斑做慢速李萨如运动。尺寸按调研报告工况设定: 水斑尺度小于
    // 视野约 1/3 —— 若斑过大, 大核高斯会把整块斑抹进背景场, 触发方案
    // 文档所述的"中空漏检"(那是 ksize 参数问题, 不是链路问题);
    // 实机部署时按"ksize ≈ 最大水斑直径 1.5 倍"整定 ir.ksize。
    const float t = static_cast<float>(frame_counter_);
    const float fx = static_cast<float>(sim_width_);
    const float fy = static_cast<float>(sim_height_);
    add_blob(
        cv::Point2f(fx * (0.35f + 0.12f * std::sin(t * 0.015f)),
                    fy * (0.45f + 0.15f * std::cos(t * 0.011f))),
        fy * 0.040f, -50);
    add_blob(
        cv::Point2f(fx * (0.70f + 0.10f * std::cos(t * 0.009f)),
                    fy * (0.60f + 0.12f * std::sin(t * 0.013f))),
        fy * 0.030f, -45);

    // 3) 静态热斑(右上角, 模拟灯具/电缆: Diff 为负, 应被方案正确忽略)
    add_blob(cv::Point2f(fx * 0.92f, fy * 0.12f), fy * 0.05f, 50);

    // 注: 模拟帧上不叠加任何文字/边框 —— 白热图上任何强对比图案都会
    // 扰动高斯背景场, 在其周围产生正 Diff 光晕而被误检为渗水区域;
    // 画面活性由两个缓慢漂移的冷斑呈现, 新鲜度由窗格信息条 fps 呈现。
}

// ============================== 采集主循环 ==============================

void IrCameraNode::captureThread()
{
    RCLCPP_INFO(
        get_logger(), "红外采集线程启动(%s)",
        sim_mode_ ? "模拟模式" : "UVC 真机模式");

    // JPEG 编码输出缓冲(线程内复用, 避免每帧重新分配)
    std::vector<unsigned char> jpeg_buffer;

    while (rclcpp::ok() && running_)
    {
        // ---- 异常安全护栏: 取帧/编码/发布任何一步抛异常(如 OpenCV 的
        //      cv::Exception)都只记录详细日志并继续, 线程绝不带异常逃逸
        //      (逃逸会 std::terminate 拖死整个进程)。 ----
        try
        {
            captureOnce(jpeg_buffer);
        }
        catch (const cv::Exception & e)
        {
            ++stat_fail_;
            RCLCPP_ERROR(
                get_logger(),
                "红外采集单次处理异常(cv::Exception): %s | code=%d | "
                "func=%s | file=%s:%d | 累计失败%lu 次, 继续循环",
                e.what(), e.code, e.func.c_str(), e.file.c_str(), e.line,
                static_cast<unsigned long>(stat_fail_));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (const std::exception & e)
        {
            ++stat_fail_;
            RCLCPP_ERROR(
                get_logger(),
                "红外采集单次处理异常(std::exception): %s | 累计失败%lu 次, "
                "继续循环",
                e.what(), static_cast<unsigned long>(stat_fail_));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (...)
        {
            ++stat_fail_;
            RCLCPP_ERROR(
                get_logger(),
                "红外采集单次处理未知异常(非 std::exception), 累计失败%lu 次, "
                "继续循环",
                static_cast<unsigned long>(stat_fail_));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    RCLCPP_INFO(get_logger(), "红外采集线程退出");
}

void IrCameraNode::captureOnce(std::vector<unsigned char> & jpeg_buffer)
{
    // ---- 周期健康报告(每 30s): 让人随时知道红外活没活 ----
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady >= next_report_)
    {
        const double elapsed = std::chrono::duration<double>(
            now_steady - (next_report_ -
                          std::chrono::duration_cast<
                              std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(kHealthReportSec))))
                                   .count();
        const double hz =
            elapsed > 0.0 ? stat_ok_ * 1.0 / elapsed : 0.0;
        RCLCPP_INFO(
            get_logger(),
            "红外健康报告: 完整帧%lu (%.1ffps) | 失败%lu | 重开%lu%s",
            static_cast<unsigned long>(stat_ok_), hz,
            static_cast<unsigned long>(stat_fail_),
            static_cast<unsigned long>(stat_reopen_),
            opened_ ? "" : " ⚠ 当前离线等待接入");
        next_report_ = now_steady +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(kHealthReportSec));
    }

    // ---- [离线] 状态: 重试打开(热插拔接入点) ----
    if (!opened_)
    {
        if (tryOpenDevice())
        {
            opened_ = true;
            fail_count_ = 0;
            ++stat_reopen_;
            RCLCPP_INFO(
                get_logger(),
                "红外相机已接入(第%lu次打开), 开始取流",
                static_cast<unsigned long>(stat_reopen_));
            return;
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), kWaitingLogThrottleMs,
            "红外相机未接入(sim离线窗/设备不存在), 每 1s 重试 ...");
        std::this_thread::sleep_for(
            std::chrono::duration<double>(kReopenIntervalSec));
        return;
    }

    // ---- [在线] 状态: 取一帧 ----
    cv::Mat gray;
    if (!grabGrayFrame(gray) || gray.empty())
    {
        ++stat_fail_;
        ++fail_count_;
        // 连续失败达上限 -> 判定离线, 关闭设备, 回到 [离线] 重试
        if (fail_count_ >= grab_fail_limit_)
        {
            RCLCPP_WARN(
                get_logger(),
                "红外相机连续 %d 次取帧失败, 判定离线, 关闭设备等待重新接入 ...",
                fail_count_);
            closeDevice();
            fail_count_ = 0;
        }
        else
        {
            // 偶发失败: 小睡后重试(与真机坏帧节奏一致)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return;
    }
    fail_count_ = 0;
    ++stat_ok_;

    // ---- 限帧: 发布帧率不超过 frame_rate_(0 = 跟随设备) ----
    if (frame_rate_ > 0.0)
    {
        if (now_steady < next_publish_time_)
        {
            // 提前到达: 睡到发布时刻(不吞帧, 只降频)
            std::this_thread::sleep_until(next_publish_time_);
        }
        next_publish_time_ +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / frame_rate_));
        // 落后太多时重置节拍(避免长时间离线后连发补帧)
        if (std::chrono::steady_clock::now() > next_publish_time_ +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(2.0)))
        {
            next_publish_time_ = std::chrono::steady_clock::now();
        }
    }

    // ---- JPEG 编码 + 发布(唯一话题) ----
    const std::vector<int> encode_params = {
        cv::IMWRITE_JPEG_QUALITY, jpeg_quality_.load()};
    if (!cv::imencode(".jpg", gray, jpeg_buffer, encode_params))
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "红外帧 JPEG 编码失败");
        return;
    }
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data = std::move(jpeg_buffer);
    pub_->publish(msg);
}

}  // namespace ir_camera_driver

// ----------------------------------------------------------------------
// 注册为 rclcpp 组件: 既能独立运行, 也能被 composition 容器加载。
// ----------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ir_camera_driver::IrCameraNode)
