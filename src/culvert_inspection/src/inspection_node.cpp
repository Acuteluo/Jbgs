// 文件: inspection_node.cpp
// 作用: 巡检通信协议节点的 rclcpp 薄封装 —— 协议逻辑全部在
//       inspection_fsm.hpp(InspectionFsm, 无 ROS 依赖, 可 gtest 单测),
//       本文件只负责: 参数装载/话题订阅发布/线程串行化/日志桥接。
//
// 线程模型(MultiThreadedExecutor):
//   - 左/右图像、指令、看门狗各自独立回调组并发进入;
//   - 图像"空闲快路径"只做原子到达计数(无锁);
//   - Armed 之后的协议处理由 state_mutex_ 串行化;
//   - 指令回调用 try_lock: 拿不到锁 = 正在写盘(Saving), 视为
//     "处理中再次收到 0x01" -> 节流告警并忽略, 绝不排队成第二次巡检。

#include "culvert_inspection/inspection_node.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/qos.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

namespace culvert_inspection
{

namespace
{

/// 相对路径解析: JBGS_ROOT(run.sh 导出的工程根)为基准, 未设置退化为 cwd。
std::string ResolvePath(const std::string & path)
{
    namespace fs = std::filesystem;
    if (path.empty() || fs::path(path).is_absolute())
    {
        return path;
    }
    const char * root = getenv("JBGS_ROOT");
    if (root != nullptr && *root != '\0')
    {
        return (fs::path(root) / path).string();
    }
    return (fs::current_path() / path).string();
}

/// 生产用解码校验: OpenCV imdecode 非空 = 有效帧。
bool DecodeJpeg(const uint8_t * data, size_t size)
{
    if (data == nullptr || size == 0)
    {
        return false;
    }
    const cv::Mat bytes(
        1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t *>(data));
    return !cv::imdecode(bytes, cv::IMREAD_COLOR).empty();
}

/// 生产用写盘: 目录确保存在 -> 二进制写入 -> flush 校验。
bool WriteJpegFile(
    rclcpp::Logger lg, const std::string & path,
    const uint8_t * data, size_t size)
{
    namespace fs = std::filesystem;
    if (size == 0)
    {
        RCLCPP_ERROR(lg, "空数据, 拒绝写入 %s", path.c_str());
        return false;
    }
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f.is_open())
    {
        RCLCPP_ERROR(lg, "无法打开文件 %s 写入(权限/路径错误)", path.c_str());
        return false;
    }
    f.write(reinterpret_cast<const char *>(data),
            static_cast<std::streamsize>(size));
    f.flush();
    if (!f.good())
    {
        RCLCPP_ERROR(lg, "写入 %s 中途失败", path.c_str());
        return false;
    }
    return true;
}

}  // namespace

const char * InspectionNode::StateName(int s)
{
    return fsmStateName(static_cast<FsmState>(s));
}

InspectionNode::InspectionNode()
: Node("inspection_node")
{
    // ---- 参数(可在 launch.json / launch 参数中覆盖) ----
    in_trulyworking_ = declare_parameter("in_trulyworking", false);
    input_topic_ = declare_parameter("input_topic", "/vision_capture_cmd");
    ack_topic_ = declare_parameter("ack_topic", "/vision_capture_status");
    status_rate_hz_ = declare_parameter("status_rate_hz", 20.0);
    annot_gap_ms_ = declare_parameter("annot_gap_ms", 500.0);
    left_annotated_topic_ = declare_parameter(
        "left_annotated_topic", "/core_node/left_annotated");
    right_annotated_topic_ = declare_parameter(
        "right_annotated_topic", "/core_node/right_annotated");
    const double frame_timeout =
        declare_parameter("frame_timeout_sec", 3.0);
    const int target_index = declare_parameter("target_frame_index", 2);
    const std::string save_dir =
        ResolvePath(declare_parameter("save_dir", "run_save"));
    use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", true);

    const std::string left_topic =
        declare_parameter("left_topic", "/left_camera/image_raw/compressed");
    const std::string right_topic =
        declare_parameter("right_topic", "/right_camera/image_raw/compressed");

    RCLCPP_INFO(
        get_logger(),
        "巡检协议节点: in_trulyworking=%s 输入=%s 确认=%s 保存目录=%s "
        "目标帧=t 后第 %d 帧 超时=%.1fs",
        in_trulyworking_ ? "true" : "false", input_topic_.c_str(),
        ack_topic_.c_str(), save_dir.c_str(), target_index, frame_timeout);

    // ---- 非巡检模式: 不订阅协议话题, 不处理, 不保存任何巡检图片 ----
    if (!in_trulyworking_)
    {
        RCLCPP_WARN(
            get_logger(),
            "in_trulyworking=false: 不订阅 %s, 不处理巡检协议, "
            "不保存巡检图片",
            input_topic_.c_str());
        return;
    }

    // ---- 组装状态机(注入 ROS 侧实现) ----
    InspectionFsm::Params p;
    p.target_frame_index = target_index;
    p.frame_timeout_sec = frame_timeout;
    p.save_dir = save_dir;
    p.annot_gap_ms = annot_gap_ms_;
    p.raw_subdir = "raw";            // 原图子目录
    p.annotated_subdir = "annotated";  // 标注图子目录

    auto lg = get_logger();
    fsm_ = std::make_unique<InspectionFsm>(
        p, "left", "right",
        DecodeJpeg,
        [lg](const std::string & path, const uint8_t * data, size_t size) {
            return WriteJpegFile(lg, path, data, size);
        },
        [](const std::string & path) {
            std::error_code ec;
            return std::filesystem::remove(path, ec);
        },
        [this]() {
            // 0x02 由协议状态定时器发出(done_await_zero 期间字节=0x02)
            done_await_zero_ = true;
        },
        []() { return std::chrono::steady_clock::now(); },
        []() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        },
        [this](const char * level, const std::string & text) {
            if (std::string(level) == "ERROR")
            {
                RCLCPP_ERROR(get_logger(), "%s", text.c_str());
            }
            else if (std::string(level) == "WARN")
            {
                RCLCPP_WARN(get_logger(), "%s", text.c_str());
            }
            else
            {
                RCLCPP_INFO(get_logger(), "%s", text.c_str());
            }
        });

    // ---- 订阅/发布(左右图像独立回调组 => 解码互不阻塞) ----
    const rclcpp::QoS qos = use_sensor_data_qos_ ?
        static_cast<rclcpp::QoS>(rclcpp::SensorDataQoS()) :
        rclcpp::SystemDefaultsQoS();

    left_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    right_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opt_left;
    opt_left.callback_group = left_cb_group_;
    rclcpp::SubscriptionOptions opt_right;
    opt_right.callback_group = right_cb_group_;
    rclcpp::SubscriptionOptions opt_cmd;
    opt_cmd.callback_group = cmd_cb_group_;

    left_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        left_topic, qos,
        [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            ImageCallback(fsm_->left(), std::move(msg));
        }, opt_left);
    right_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        right_topic, qos,
        [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            ImageCallback(fsm_->right(), std::move(msg));
        }, opt_right);

    cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
        input_topic_, rclcpp::QoS(10),
        [this](std_msgs::msg::UInt8::SharedPtr msg) { CommandCallback(msg); },
        opt_cmd);

    // 协议状态话题: 0x00 空闲 / 0x01 保存中 / 0x02 拍完(即协议里的"回传")
    status_pub_ = create_publisher<std_msgs::msg::UInt8>(ack_topic_, rclcpp::QoS(10));
    status_str_pub_ = create_publisher<std_msgs::msg::String>(
        "~/status", rclcpp::QoS(10));

    // 协议状态定时发布(可调频率)
    const auto status_period = std::chrono::milliseconds(
        static_cast<int64_t>(1000.0 / std::max(status_rate_hz_, 1.0)));
    status_timer_ = create_wall_timer(
        status_period, [this]() { StatusTimer(); });

    // 看门狗: 10Hz 超时检查 + 调试状态发布。
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { WatchdogTimer(); });

    // 标注图订阅(culvert_core 发布, 时间戳与原图配对)
    left_ann_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        left_annotated_topic_, qos,
        [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            AnnotatedCallback(fsm_->left(), std::move(msg));
        },
        opt_left);
    right_ann_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        right_annotated_topic_, qos,
        [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            AnnotatedCallback(fsm_->right(), std::move(msg));
        },
        opt_right);

    // 保存目录提前建好(写盘阶段还会再校验并报错)。
    std::error_code ec;
    std::filesystem::create_directories(save_dir, ec);
}

// ============================== 指令 ==============================

void InspectionNode::CommandCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (msg->data != 0x01)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *this, 5000,
            "收到未知巡检指令 0x%02X(仅支持 0x01), 忽略", msg->data);
        return;
    }

    last_cmd_level_.store(msg->data, std::memory_order_relaxed);
    // 导航回 0x00 = 上一站流程结束, 解除门控(之后的 0x01 才是新周期)
    if (msg->data == 0x00)
    {
        done_await_zero_ = false;
    }
    // 完成后残留的 0x01(导航尚未切回 0x00)必须忽略, 防止误触发下一站
    if (done_await_zero_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *this, 2000,
            "已完成等待导航回 0x00, 忽略残留 0x01");
        return;
    }

    // try_lock: 拿不到 = 正在写盘(Saving 瞬态) => "处理中再次收到 0x01",
    // 节流告警并忽略, 不排队、不误发第二次 0x02。
    if (!state_mutex_.try_lock())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *this, 2000,
            "巡检保存进行中再次收到 0x01, 忽略(每个 0x01 只触发一次保存)");
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_, std::adopt_lock);
    fsm_->onCommand(msg->data);
}

// ============================== 图像 ==============================

void InspectionNode::ImageCallback(
    InspectionFsm::Side & side,
    sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 统一取锁交状态机(无无锁快路径): 计数、arm()、帧处理三者被
    // state_mutex_ 完全串行化, 从构造上关闭"帧落在 arm 执行中间被
    // 跳过"的竞态窗口。60 帧/秒量级下锁开销可忽略。
    InspectionFrame frame;
    frame.data = msg->data.data();
    frame.size = msg->data.size();
    frame.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

    std::lock_guard<std::mutex> lock(state_mutex_);
    fsm_->onFrame(side, frame);
}

// ============================== 看门狗 ==============================

void InspectionNode::WatchdogTimer()
{
    // 状态发布(有订阅者才发, 空载零流量)。
    if (status_str_pub_ && status_str_pub_->get_subscription_count() > 0)
    {
        std_msgs::msg::String st;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "state=%s cycle=%lu left_arrived=%lu "
                      "right_arrived=%lu",
                      StateName(static_cast<int>(fsm_->state())),
                      static_cast<unsigned long>(fsm_->cycle()),
                      static_cast<unsigned long>(
                          fsm_->left().arrived.load(std::memory_order_relaxed)),
                      static_cast<unsigned long>(
                          fsm_->right().arrived.load(std::memory_order_relaxed)));
        st.data = buf;
        status_str_pub_->publish(st);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    fsm_->checkTimeout();
}

// ============================== 协议状态发布 ==============================

void InspectionNode::StatusTimer()
{
    if (!status_pub_)
    {
        return;   // 非巡检模式
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    uint8_t byte = 0x00;
    if (done_await_zero_)
    {
        byte = 0x02;   // 已完成, 等导航回 0x00
    }
    else if (fsm_->state() != FsmState::kIdle)
    {
        byte = 0x01;   // 正在等待/保存图片
    }
    std_msgs::msg::UInt8 msg;
    msg.data = byte;
    status_pub_->publish(msg);
}

// ============================== 标注图回调 ==============================

void InspectionNode::AnnotatedCallback(
    InspectionFsm::Side & side,
    sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    if (fsm_->state() != FsmState::kArmed)
    {
        return;   // 只在等待保存期间配对
    }
    InspectionFrame frame;
    frame.data = msg->data.data();
    frame.size = msg->data.size();
    frame.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    std::lock_guard<std::mutex> lock(state_mutex_);
    fsm_->onAnnotatedFrame(side, frame);
}

InspectionNode::~InspectionNode() = default;

}  // namespace culvert_inspection

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<culvert_inspection::InspectionNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
