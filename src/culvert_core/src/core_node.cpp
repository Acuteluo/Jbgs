// Copyright (c) 2024, culvert_core contributors
//
// 文件: core_node.cpp
// 作用: 涵洞检测核心节点实现(多源感知 + 三画面同屏显示版)。
//
// 数据流(全部 compressed 模式, 话题精简: 4 条数据话题):
//   galaxy_camera_dual_node --/left_camera/image_raw/compressed--->  左回调
//                           --/right_camera/image_raw/compressed--> 右回调
//   ir_camera_driver_node   --/ir_camera/image_raw/compressed---->  红外回调
//   tas_sensor_driver_node  --/sensor/env------------------------>  环境回调
//
// 回调只做"JPEG 解码 + 写最新帧缓存"; 重活全在独立线程:
//   - 左/右 YOLO 检测线程(各自独立 YoloDetector 实例, 并行推理);
//   - 红外渗水检测线程(IrSeepageDetector 背景差分, 来自
//     Culvert-Visual-Inspection-main 的传统方案);
//   - 同屏显示线程(三窗格合成, 传感器叠总图左上角, 单窗口 imshow)。
//
// 帧率与线程冲突的处理策略(为什么不会互相拖慢):
//   1. 三路图像订阅各自独立回调组, MultiThreadedExecutor 并发调度,
//      一路解码(约几十毫秒)不会排队阻塞其他路;
//   2. 回调 -> 检测线程 -> 显示线程之间全部"最新帧覆盖"交接:
//      慢的消费者只丢帧, 不回压生产者(相机帧率不受下游影响);
//   3. 锁的粒度只覆盖指针交接(小拷贝), 推理/GUI 全部在锁外;
//   4. 任何一路断流(热插拔期间)只是该路画面停更, 显示端显示占位提示,
//      其他路照常; 恢复后画面自动续上, 无需重启任何节点。

#include "culvert_core/core_node.hpp"

#include <rclcpp/qos.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>     // std::array: 显示线程的三窗格表
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <typeinfo>
#include <utility>   // std::pair
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace culvert_core
{

namespace
{
/// 每次循环无新帧时的轮询小睡(检测线程)。
constexpr auto kDetectPollInterval = std::chrono::milliseconds(10);

/// 显示循环的节拍间隔由 display_fps_ 计算, 这里是下限保护。
constexpr auto kDisplayMinInterval = std::chrono::milliseconds(5);

/// 红外时间一致性滑窗的断流复位间隔(秒): 两次处理帧间隔超过该值
/// (热插拔离线/长时间断流)视为场景不连续, 复位滑窗历史防止"假确认"。
constexpr double kIrTemporalResetGapSec = 1.0;

/// imshow 窗口标题(单窗口同屏)。
constexpr const char * kWindowName = "culvert_monitor";

/// 窗格顶部信息条高度(两行: 标题 + src/det 帧率; 全屏下清晰可读)。
constexpr int kPaneHeaderHeight = 52;

/// 等比适配到窗格: 图像缩放到"恰好装进"窗格(不变形), 居中放在深灰
/// 底上。全屏模式下窗格为竖长条(与屏幕同宽高比), 相机 4:3 图像上下
/// 留边属于预期布局。返回缩放系数与内容偏移, 供坐标映射(红外轮廓)。
void FitToPane(
    const cv::Mat & src, int pane_w, int pane_h, cv::Mat & dst,
    double * scale_out = nullptr, int * dx_out = nullptr, int * dy_out = nullptr)
{
    dst = cv::Mat(pane_h, pane_w, CV_8UC3, cv::Scalar(28, 28, 28));
    if (src.empty())
    {
        if (scale_out) { *scale_out = 0.0; }
        if (dx_out) { *dx_out = 0; }
        if (dy_out) { *dy_out = 0; }
        return;
    }
    const double s = std::min(
        static_cast<double>(pane_w) / std::max(src.cols, 1),
        static_cast<double>(pane_h) / std::max(src.rows, 1));
    const int fitted_w = std::max(
        1, static_cast<int>(std::round(src.cols * s)));
    const int fitted_h = std::max(
        1, static_cast<int>(std::round(src.rows * s)));
    cv::Mat fitted;
    cv::resize(src, fitted, cv::Size(fitted_w, fitted_h),
               0.0, 0.0, s < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    const int dx = (pane_w - fitted_w) / 2;
    const int dy = (pane_h - fitted_h) / 2;
    fitted.copyTo(dst(cv::Rect(dx, dy, fitted_w, fitted_h)));
    if (scale_out) { *scale_out = s; }
    if (dx_out) { *dx_out = dx; }
    if (dy_out) { *dy_out = dy; }
}

/// 相对路径解析: 以 JBGS_ROOT 环境变量(run.sh 导出)为基准,
/// 未设置时退化为进程 cwd。绝对路径原样返回。
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
}  // namespace

// ============================== 构造 ==============================

CoreNode::CoreNode()
: Node("core_node")
{
    RCLCPP_INFO(get_logger(), "CoreNode 开始初始化");

    // 1. 参数
    InitParams();
    save_dir_ = ResolvePath(save_dir_);
    yolo_model_path_ = ResolvePath(yolo_model_path_);

    // 全屏模式: 探测屏幕分辨率, 窗格高度按画布:屏幕 = 1:1 反推
    // (画布宽 = 3*pane_w + 2*gap, 与屏幕同比 => 铺满无黑边)。
    if (fullscreen_)
    {
        int screen_w = 0;
        int screen_h = 0;
        if (auto * fp = popen("xrandr --current 2>/dev/null", "r"))
        {
            char line[256];
            while (fgets(line, sizeof(line), fp) != nullptr)
            {
                // 形如 "... current 2880 x 1920 ..."
                const char * key = strstr(line, "current ");
                if (key != nullptr &&
                    std::sscanf(key, "current %d x %d", &screen_w,
                                &screen_h) == 2 &&
                    screen_w > 0 && screen_h > 0)
                {
                    break;
                }
            }
            pclose(fp);
        }
        if (screen_w <= 0 || screen_h <= 0)
        {
            // 探测失败(无 xrandr/无显示): 用参数或保守默认, 不阻塞
            screen_w = screen_w_ > 0 ? screen_w_ : 1920;
            screen_h = screen_h_ > 0 ? screen_h_ : 1080;
            RCLCPP_WARN(
                get_logger(),
                "屏幕分辨率探测失败(xrandr 不可用?), 使用 %dx%d",
                screen_w, screen_h);
        }
        screen_w_ = screen_w;
        screen_h_ = screen_h;
        pane_height_ = static_cast<int>(std::round(
            (3.0 * pane_width_ + 2.0 * pane_gap_) * screen_h /
            static_cast<double>(screen_w)));
        RCLCPP_INFO(
            get_logger(),
            "全屏展示: 屏幕 %dx%d, 画布 %dx%d (窗格 %dx%d, 图像等比留边)",
            screen_w_, screen_h_, 3 * pane_width_ + 2 * pane_gap_,
            pane_height_, pane_width_, pane_height_);
    }

    // 2. 三路视频源上下文的静态信息(话题名等)
    left_.name = "left";
    left_.title = "LEFT-DAHENG";
    left_.topic = "/left_camera/image_raw/compressed";
    right_.name = "right";
    right_.title = "RIGHT-DAHENG";
    right_.topic = "/right_camera/image_raw/compressed";
    ir_.name = "ir";
    ir_.title = "IR-THERMAL";
    ir_.topic = "/ir_camera/image_raw/compressed";

    // 3. 订阅/发布/定时器
    InitROS2();

    // 4. 保存目录
    std::error_code ec;
    std::filesystem::create_directories(save_dir_, ec);

    // 5. YOLO 检测器(左右各一个实例; 失败只降级为透传, 不退出)
    InitYolo();

    // 6. 红外渗水检测器(参数来自 config; 检测器线程独占, 无需加锁)
    if (ir_enable_)
    {
        ir_detector_ = std::make_unique<IrSeepageDetector>(ir_params_);
        RCLCPP_INFO(
            get_logger(),
            "红外渗水检测已启用: ksize=%d diff_thresh=%.1f "
            "morph=%d/%d min_area=%.0f temporal=%d帧/%d命中",
            ir_params_.ksize, ir_params_.diff_thresh, ir_params_.morph_ksize,
            ir_params_.morph_close_ksize, ir_params_.min_area_px,
            ir_params_.temporal_window, ir_params_.temporal_min_hits);
    }

    // 7. 启动各工作线程(异常护栏在各线程循环内部, 见各 Loop 实现)
    detect_run_ = true;
    display_run_ = true;

    left_detect_thread_ = std::thread(
        [this]()
        {
            VisibleDetectLoop(left_, pane_left_, yolo_left_, "left");
        });
    right_detect_thread_ = std::thread(
        [this]()
        {
            VisibleDetectLoop(right_, pane_right_, yolo_right_, "right");
        });
    ir_detect_thread_ = std::thread([this]() { IrDetectLoop(); });
    display_thread_ = std::thread([this]() { DisplayLoop(); });
    RCLCPP_INFO(
        get_logger(),
        "工作线程已启动: 左YOLO(%s) 右YOLO(%s) 红外渗水(%s) 同屏显示(%s)",
        yolo_left_ ? "模型OK" : "透传",
        yolo_right_ ? "模型OK" : "透传",
        ir_detector_ ? "启用" : "透传",
        show_img_ ? "开窗" : "关闭");

    // 8. 按键保存(仅 stdin 为终端时; launch 启动时由 key_forward 转发话题)
    if (::isatty(STDIN_FILENO))
    {
        key_run_ = true;
        key_thread_ = std::thread([this]() { KeyListenLoop(); });
        RCLCPP_INFO(
            get_logger(), "按键保存已启用: 按 S 键保存总图+三路帧到 %s",
            save_dir_.c_str());
    }
    else
    {
        RCLCPP_INFO(
            get_logger(),
            "stdin 非终端(ros2 launch 启动): 按键保存经话题 "
            "/core_node/save_image 触发, 保存目录 %s",
            save_dir_.c_str());
    }
}

// ============================== 参数 ==============================

void CoreNode::InitParams()
{
    // 话题(与三个驱动节点的默认发布话题一一对应)
    // 相对路径(如 run_save / models/xxx.onnx)以 JBGS_ROOT 环境变量
    // (run.sh 导出的工程根目录)解析; 未设置且无绝对路径时退化为 cwd。
    save_dir_ = declare_parameter("save_dir", "run_save");
    use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", true);

    // 同屏显示
    pane_width_ = declare_parameter("pane_width", 640);
    pane_height_ = declare_parameter("pane_height", 480);
    // 全屏展示: 窗口无边框占满整屏, 且窗格高度自动按屏幕宽高比计算
    // (画布与屏幕同比例 => 铺满无黑边; 相机图像在窗格内等比留边)。
    fullscreen_ = declare_parameter("fullscreen", true);
    // 0 = 自动探测(xrandr); 探测失败时用参数值兜底, 再不行用 1920x1080
    screen_w_ = declare_parameter("screen_width", 0);
    screen_h_ = declare_parameter("screen_height", 0);
    pane_gap_ = declare_parameter("pane_gap", 4);
    display_fps_ = declare_parameter("display_fps", 30.0);
    stale_timeout_sec_ = declare_parameter("stale_timeout_sec", 3.0);
    show_img_ = declare_parameter("show_img", true);
    // 无图形环境(无 DISPLAY)强制关闭窗口, 避免 imshow 崩溃/阻塞
    if (show_img_ && getenv("DISPLAY") == nullptr)
    {
        RCLCPP_WARN(
            get_logger(),
            "DISPLAY 未设置(无图形环境), 强制关闭同屏窗口");
        show_img_ = false;
    }

    // YOLO(左右共用模型配置)
    yolo_enable_left_ = declare_parameter("yolo.enable_left", true);
    yolo_enable_right_ = declare_parameter("yolo.enable_right", true);
    yolo_model_path_ = declare_parameter(
        "yolo.model_path", "models/crack_seg_best.onnx");
    yolo_model_type_ = declare_parameter("yolo.model_type", "seg");
    yolo_use_gpu_ = declare_parameter("yolo.use_gpu", false);
    yolo_conf_threshold_ =
        static_cast<float>(declare_parameter("yolo.conf_threshold", 0.35));
    yolo_nms_threshold_ =
        static_cast<float>(declare_parameter("yolo.nms_threshold", 0.45));
    yolo_input_size_ = declare_parameter("yolo.input_size", 640);
    yolo_max_threads_ = declare_parameter("yolo.max_threads", 8);

    // 红外渗水检测(默认值取 Culvert-Visual-Inspection-main 方案文档推荐)
    ir_enable_ = declare_parameter("ir.enable", true);
    ir_params_.ksize = declare_parameter("ir.ksize", 71);
    ir_params_.diff_thresh =
        static_cast<float>(declare_parameter("ir.diff_thresh", 15.0));
    ir_params_.morph_ksize = declare_parameter("ir.morph_ksize", 3);
    ir_params_.morph_close_ksize = declare_parameter("ir.morph_close_ksize", 21);
    // 60: 按 256x192 原生分辨率整定(实测第二渗水斑面积约 110px,
    // 旧默认 120 会把它当噪声滤掉; 256 尺度下 30~60 合理, 见方案文档)
    ir_params_.min_area_px = declare_parameter("ir.min_area_px", 60.0);
    ir_params_.min_area_ratio = declare_parameter("ir.min_area_ratio", 0.3);
    // 时间一致性: 最近 window 帧滑窗内命中 >= min_hits 才确认渗水,
    // 抗单帧误报(JPEG 块效应/AGC 抖动/飞过冷物体); window<=1 = 关闭
    ir_params_.temporal_window = declare_parameter("ir.temporal_window", 5);
    ir_params_.temporal_min_hits =
        declare_parameter("ir.temporal_min_hits", 3);
    // 标注图发布: 巡检保存用的是"模型处理完画框后的帧"
    publish_annotated_ = declare_parameter("publish_annotated", true);
    annotated_quality_ =
        declare_parameter("annotated_jpeg_quality", 85);
    // 导航协议可视化: 只订阅展示, 不干预协议
    vision_cmd_topic_ = declare_parameter(
        "vision_cmd_topic", "/vision_capture_cmd");
    vision_status_topic_ = declare_parameter(
        "vision_status_topic", "/vision_capture_status");
    ir_params_.enable_clahe = declare_parameter("ir.enable_clahe", false);
    ir_params_.clahe_clip = declare_parameter("ir.clahe_clip", 2.0);
}

void CoreNode::InitROS2()
{
    // QoS 与驱动发布端一致: sensor_data 低延迟、容忍丢帧
    const rclcpp::QoS qos = use_sensor_data_qos_ ?
        static_cast<rclcpp::QoS>(rclcpp::SensorDataQoS()) :
        rclcpp::SystemDefaultsQoS();

    // 三路图像订阅各自独立回调组 => 三路 JPEG 解码并发执行,
    // 一路卡顿(大图解码/坏帧)不会排队阻塞其他路(多线程要点)。
    // 标注图发布器(模型处理完画框后的帧; 巡检协议保存用)
    if (publish_annotated_)
    {
        const auto ann_qos = rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data);
        left_annotated_pub_ =
            create_publisher<sensor_msgs::msg::CompressedImage>(
                "/core_node/left_annotated", ann_qos);
        right_annotated_pub_ =
            create_publisher<sensor_msgs::msg::CompressedImage>(
                "/core_node/right_annotated", ann_qos);
    }

    // 导航协议可视化订阅(独立展示, 不影响巡检节点)
    const auto nav_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
        rmw_qos_profile_sensor_data);
    nav_cmd_sub_ = create_subscription<std_msgs::msg::UInt8>(
        vision_cmd_topic_, nav_qos,
        [this](std_msgs::msg::UInt8::SharedPtr m) {
            nav_cmd_.store(m->data, std::memory_order_relaxed);
        });
    nav_status_sub_ = create_subscription<std_msgs::msg::UInt8>(
        vision_status_topic_, nav_qos,
        [this](std_msgs::msg::UInt8::SharedPtr m) {
            nav_status_.store(m->data, std::memory_order_relaxed);
        });

    left_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    right_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ir_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opt_left;
    opt_left.callback_group = left_cb_group_;
    rclcpp::SubscriptionOptions opt_right;
    opt_right.callback_group = right_cb_group_;
    rclcpp::SubscriptionOptions opt_ir;
    opt_ir.callback_group = ir_cb_group_;

    left_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        left_.topic, qos,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
        {
            LeftCallback(msg);
        },
        opt_left);
    right_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        right_.topic, qos,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
        {
            RightCallback(msg);
        },
        opt_right);
    ir_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        ir_.topic, qos,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
        {
            IrCallback(msg);
        },
        opt_ir);

    // 环境数据(1Hz, 温湿度/CO2)
    env_sub_ = create_subscription<sensor_interfaces::msg::EnvData>(
        "/sensor/env", qos,
        [this](const sensor_interfaces::msg::EnvData::SharedPtr msg)
        {
            EnvCallback(msg);
        });

    // "保存一帧"请求(launch 的 key_forward 把终端 S 键转发到这里)
    save_req_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/core_node/save_image", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Empty::SharedPtr)
        {
            SaveCurrentFrame();
        });

    // 1Hz 心跳(帧率/传感器摘要)
    status_pub_ = create_publisher<std_msgs::msg::String>("/core_node/status", qos);
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]()
        {
            PublishStatus();
        });
}

// ============================== YOLO 初始化 ==============================

void CoreNode::InitYolo()
{
    // 模型类型: "seg" = 框 + 分割描边(默认); 其他 = 仅框
    const YoloDetector::ModelType model_type =
        (yolo_model_type_ == "seg") ?
        YoloDetector::ModelType::Seg : YoloDetector::ModelType::Detect;

    // 限制 OpenCV 线程池: 默认吃满所有核心会饿住解码/显示线程
    if (yolo_max_threads_ > 0 && cv::getNumThreads() > yolo_max_threads_)
    {
        cv::setNumThreads(yolo_max_threads_);
        RCLCPP_INFO(get_logger(), "OpenCV 线程池限制为 %d", yolo_max_threads_);
    }

    // 左右各创建一个独立实例: cv::dnn::Net 非线程安全, 共享单实例
    // 会让两路推理串行排队; 双实例各占各的网络内存, 真正并行。
    auto make_detector = [this, model_type](
                             const char * tag) -> std::unique_ptr<YoloDetector>
    {
        try
        {
            auto det = std::make_unique<YoloDetector>(
                yolo_model_path_, yolo_use_gpu_, model_type);
            if (!det->IsReady())
            {
                RCLCPP_ERROR(
                    get_logger(), "%s YOLO 模型加载失败: %s (该路降级为透传)",
                    tag, yolo_model_path_.c_str());
                return nullptr;
            }
            det->score_threshold_ = yolo_conf_threshold_;
            det->nms_threshold_ = yolo_nms_threshold_;
            det->input_size_ = yolo_input_size_;
            return det;
        }
        catch (const std::exception & e)
        {
            // 模型缺失/损坏只降级不退出: 相机链路与显示照常工作
            RCLCPP_ERROR(
                get_logger(), "%s YoloDetector 创建异常: %s (该路降级为透传)",
                tag, e.what());
            return nullptr;
        }
    };

    if (yolo_enable_left_)
    {
        yolo_left_ = make_detector("左相机");
    }
    if (yolo_enable_right_)
    {
        yolo_right_ = make_detector("右相机");
    }
}

// ============================== 图像回调 ==============================

void CoreNode::CompressedImageCallback(
    StreamContext & stream, const sensor_msgs::msg::CompressedImage & msg)
{
    // 异常安全: 坏 JPEG/OpenCV 内部异常只记录详细日志并丢弃本帧,
    // 不允许异常逃出回调(会 std::terminate 拖死进程)。
    try
    {
        const cv::Mat encoded(
            1, static_cast<int>(msg.data.size()), CV_8UC1,
            const_cast<unsigned char *>(msg.data.data()));
        cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (decoded.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "[%s] JPEG 解码失败(数据 %zu 字节), 丢弃本帧",
                stream.name.c_str(), msg.data.size());
            return;
        }

        // 红外 JPEG 可能是单通道灰度: 统一转 BGR, 后续画框/合成统一处理
        if (decoded.channels() == 1)
        {
            cv::Mat bgr;
            cv::cvtColor(decoded, bgr, cv::COLOR_GRAY2BGR);
            decoded = bgr;
        }

        // 写入最新帧缓存(锁内小拷贝; 只保留最新, 消费慢自动丢旧帧)
        std::lock_guard<std::mutex> lock(stream.mutex);
        decoded.copyTo(stream.latest_bgr);
        stream.latest_stamp = msg.header.stamp;
        // 原图取帧率 EMA(帧间到达间隔): 与检测输出帧率独立,
        // 模型消费慢/丢帧不影响该值 —— 窗格信息条实时显示用。
        const auto now_arrival = std::chrono::steady_clock::now();
        const double dt_arr = std::chrono::duration<double>(
            now_arrival - stream.latest_arrival).count();
        if (stream.seq > 0 && dt_arr > 0.0)
        {
            const double inst = 1.0 / dt_arr;
            const double prev =
                stream.src_fps.load(std::memory_order_relaxed);
            // 间隔超过 2s(断连后恢复)直接跳变, 不做缓慢 EMA 爬升
            stream.src_fps.store(
                (prev > 0.0 && dt_arr < 2.0) ?
                    (0.7 * prev + 0.3 * inst) : inst,
                std::memory_order_relaxed);
        }
        stream.latest_arrival = now_arrival;
        ++stream.seq;
        stream.count.fetch_add(1, std::memory_order_relaxed);
    }
    catch (const cv::Exception & e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "[%s] 图像回调异常(cv::Exception): %s | code=%d | func=%s | "
            "file=%s:%d, 丢弃本帧",
            stream.name.c_str(), e.what(), e.code, e.func.c_str(),
            e.file.c_str(), e.line);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            get_logger(), "[%s] 图像回调异常(std::exception): %s, 丢弃本帧",
            stream.name.c_str(), e.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(
            get_logger(), "[%s] 图像回调未知异常, 丢弃本帧", stream.name.c_str());
    }
}

void CoreNode::LeftCallback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    CompressedImageCallback(left_, *msg);
}

void CoreNode::RightCallback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    CompressedImageCallback(right_, *msg);
}

void CoreNode::IrCallback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    CompressedImageCallback(ir_, *msg);
}

// ============================== 帧交接工具 ==============================

bool CoreNode::FetchLatest(
    StreamContext & stream, uint64_t & last_seq, cv::Mat & frame)
{
    std::lock_guard<std::mutex> lock(stream.mutex);
    if (stream.seq == last_seq || stream.latest_bgr.empty())
    {
        return false;   // 无新帧: 检测线程据此小睡等待
    }
    // 锁内拷贝(1280x960 约 3.7MB, 拷贝 <2ms): 保持"锁内只做小工作"
    stream.latest_bgr.copyTo(frame);
    last_seq = stream.seq;
    return true;
}

void CoreNode::SubmitPane(PaneContext & pane, cv::Mat & annotated, uint64_t seq)
{
    std::lock_guard<std::mutex> lock(pane.mutex);
    cv::swap(pane.annotated, annotated);
    pane.seq = seq;
}

// ============================== 检测线程 ==============================

void CoreNode::VisibleDetectLoop(
    StreamContext & stream, PaneContext & pane,
    std::unique_ptr<YoloDetector> & detector, const std::string & window_tag)
{
    RCLCPP_INFO(get_logger(), "[%s] YOLO 检测线程运行中", stream.name.c_str());

    uint64_t last_seq = 0;
    double fps = 0.0;   // 本线程处理帧率(EMA 平滑)
    auto prev_time = std::chrono::steady_clock::now();

    while (rclcpp::ok() && detect_run_)
    {
        // 单次处理包在 lambda 里, 由统一异常护栏包裹:
        // 推理/画框抛异常只记录并继续, 线程绝不退出。
        cv::Mat frame;
        bool has_frame = false;
        try
        {
            has_frame = FetchLatest(stream, last_seq, frame);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_logger(), "[%s] 取帧异常: %s",
                         stream.name.c_str(), e.what());
        }
        if (!has_frame)
        {
            std::this_thread::sleep_for(kDetectPollInterval);
            continue;
        }

        try
        {
            const auto iter_start = std::chrono::steady_clock::now();

            // 1) 等比适配到窗格: 推理与画框都在窗格坐标系下进行,
            //    框坐标与显示像素一一对应(旧版"全分辨率画框后缩放"
            //    会导致框错位, 此处从根上避免)。全屏竖长窗格下上下
            //    留边属于预期布局(不变形优先)。
            cv::Mat pane_img;
            FitToPane(frame, pane_width_, pane_height_, pane_img);

            // 2) 推理(detector 为空 = 透传模式, 只出画面不检测)
            double infer_ms = 0.0;
            size_t n_cracks = 0;
            if (detector && detector->IsReady())
            {
                const auto infer_start = std::chrono::steady_clock::now();
                std::vector<CrackObject> results = detector->Detect(pane_img);
                infer_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - infer_start).count();
                n_cracks = results.size();

                // 3) 画框: 红框 + 置信度标签, seg 模型额外青色掩膜描边
                for (const auto & obj : results)
                {
                    YoloCrack crack(obj.box, obj.confidence, obj.class_id);
                    crack.SetMask(obj.mask);
                    crack.Draw(pane_img);
                }
            }

            // 4) 处理帧率(EMA 平滑)
            const auto now_time = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(
                now_time - prev_time).count();
            prev_time = now_time;
            if (dt > 0.0)
            {
                fps = (fps > 0.0) ? (0.7 * fps + 0.3 * (1.0 / dt)) : (1.0 / dt);
            }

            // 5) 窗格信息条(标题 + 原图取帧率 + 检测输出帧率 + 检测摘要)
            //    src = 取原相机原图帧率(回调 EMA, 实时); det = 本检测线程
            //    输出帧率(推理完成画框后交显示的速率)。两者独立:
            //    模型消费慢时 src 保持, det 下降 —— 一眼可辨瓶颈在哪级。
            const std::string info =
                cv::format("src=%.1f det=%.1f fps cracks=%zu infer=%.0fms",
                           stream.src_fps.load(std::memory_order_relaxed),
                           fps, n_cracks, infer_ms);
            DrawPaneHeader(pane_img, stream.title, info);

            // 5.5) 发布标注图(模型处理完画框后的帧), 供巡检协议保存;
            //      JPEG 编码在本检测线程内完成, 与显示/采集互不阻塞。
            if (publish_annotated_ && !pane_img.empty())
            {
                auto & ann_pub = (&stream == &left_) ? left_annotated_pub_
                                                     : right_annotated_pub_;
                // 仅在有订阅者时才做 JPEG 编码(无订阅零开销)
                if (ann_pub && ann_pub->get_subscription_count() > 0)
                {
                    std::vector<uint8_t> buf;
                    std::vector<int> qp = {cv::IMWRITE_JPEG_QUALITY,
                                           annotated_quality_};
                    if (cv::imencode(".jpg", pane_img, buf, qp))
                    {
                        sensor_msgs::msg::CompressedImage msg;
                        msg.header.stamp = stream.latest_stamp;
                        msg.header.frame_id = stream.name;
                        msg.format = "jpeg";
                        msg.data = std::move(buf);
                        ann_pub->publish(msg);
                    }
                }
            }

            // 6) 交给显示线程(锁内 swap, 只保留最新)
            SubmitPane(pane, pane_img, last_seq);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "[%s](%s) src=%.1f det=%.1f fps cracks=%zu infer=%.0fms",
                stream.name.c_str(), window_tag.c_str(),
                stream.src_fps.load(std::memory_order_relaxed),
                fps, n_cracks, infer_ms);
        }
        catch (const cv::Exception & e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[%s] 检测线程异常(cv::Exception): %s | code=%d | func=%s | "
                "file=%s:%d, 继续循环",
                stream.name.c_str(), e.what(), e.code, e.func.c_str(),
                e.file.c_str(), e.line);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(
                get_logger(), "[%s] 检测线程异常(std::exception): %s, 继续循环",
                stream.name.c_str(), e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(
                get_logger(), "[%s] 检测线程未知异常, 继续循环",
                stream.name.c_str());
        }

        // 小睡兜底: 推理本身已耗时, 这里只防 0 间隔空转
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    RCLCPP_INFO(get_logger(), "[%s] YOLO 检测线程退出", stream.name.c_str());
}

void CoreNode::IrDetectLoop()
{
    RCLCPP_INFO(get_logger(), "[ir] 红外渗水检测线程运行中");

    uint64_t last_seq = 0;
    double fps = 0.0;
    auto prev_time = std::chrono::steady_clock::now();
    // 上次处理的帧到达时刻(检测线程视角): 用于断流复位时间一致性滑窗。
    // epoch 为 0 表示尚未处理过任何帧。
    auto last_frame_time = std::chrono::steady_clock::time_point{};

    while (rclcpp::ok() && detect_run_)
    {
        cv::Mat frame;
        bool has_frame = false;
        try
        {
            has_frame = FetchLatest(ir_, last_seq, frame);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_logger(), "[ir] 取帧异常: %s", e.what());
        }
        if (!has_frame)
        {
            std::this_thread::sleep_for(kDetectPollInterval);
            continue;
        }

        try
        {
            // 0) 断流复位: 距上帧超过 kIrTemporalResetGapSec(热插拔离线/
            //    长时间断流)视为场景不连续, 清空时间一致性滑窗 —— 滑窗里
            //    的旧帧与当前画面已无关, 继续沿用会造成"假确认"。
            const auto now_frame = std::chrono::steady_clock::now();
            if (last_frame_time.time_since_epoch().count() != 0 && ir_detector_)
            {
                const double gap = std::chrono::duration<double>(
                    now_frame - last_frame_time).count();
                if (gap > kIrTemporalResetGapSec)
                {
                    ir_detector_->reset();
                    RCLCPP_INFO(
                        get_logger(),
                        "[ir] 相隔 %.1fs 无新帧, 已复位时间一致性滑窗", gap);
                }
            }
            last_frame_time = now_frame;

            // 1) 在机芯"原生分辨率"上检测(如 256x192), 不要先放大再检测:
            //    上采样会摊平冷斑与背景场的灰度差(实测 Diff 峰值 36 -> 20,
            //    贴着阈值导致漏检), 且 ksize 参数与原生分辨率绑定才可移植。
            const auto infer_start = std::chrono::steady_clock::now();
            IrSeepageResult result;
            if (ir_detector_)
            {
                // detect 带滑窗内部状态, 本线程内串行调用无需加锁
                result = ir_detector_->detect(frame);
            }
            const double infer_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - infer_start).count();
            const size_t n_regions = result.regions.size();

            // 2) 原始帧放大到窗格尺寸, 检测结果(轮廓/外接框)按同比例
            //    等比适配到窗格坐标后画框 —— 显示像素与标注一一对应
            //    (等比缩放 + 居中偏移, 轮廓坐标按下式映射)。
            double fit_scale = 1.0;
            int fit_dx = 0;
            int fit_dy = 0;
            cv::Mat pane_img;
            FitToPane(frame, pane_width_, pane_height_, pane_img,
                      &fit_scale, &fit_dx, &fit_dy);
            // 灰度帧转 BGR, 供画黄色轮廓
            if (pane_img.channels() == 1)
            {
                cv::cvtColor(pane_img, pane_img, cv::COLOR_GRAY2BGR);
            }
            for (auto & region : result.regions)
            {
                region.bbox.x = static_cast<int>(
                    region.bbox.x * fit_scale) + fit_dx;
                region.bbox.y = static_cast<int>(
                    region.bbox.y * fit_scale) + fit_dy;
                region.bbox.width = static_cast<int>(
                    region.bbox.width * fit_scale);
                region.bbox.height = static_cast<int>(
                    region.bbox.height * fit_scale);
                for (auto & pt : region.contour)
                {
                    pt.x = static_cast<int>(pt.x * fit_scale) + fit_dx;
                    pt.y = static_cast<int>(pt.y * fit_scale) + fit_dy;
                }
            }
            IrSeepageDetector::drawOverlay(pane_img, result);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "[ir] src=%.1f det=%.1f fps 渗水区域=%zu infer=%.1fms",
                ir_.src_fps.load(std::memory_order_relaxed),
                fps, n_regions, infer_ms);

            const auto now_time = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(
                now_time - prev_time).count();
            prev_time = now_time;
            if (dt > 0.0)
            {
                fps = (fps > 0.0) ? (0.7 * fps + 0.3 * (1.0 / dt)) : (1.0 / dt);
            }

            const std::string info =
                cv::format("src=%.1f det=%.1f fps seepage=%zu",
                           ir_.src_fps.load(std::memory_order_relaxed),
                           fps, n_regions);
            DrawPaneHeader(pane_img, ir_.title, info);

            SubmitPane(pane_ir_, pane_img, last_seq);
        }
        catch (const cv::Exception & e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[ir] 渗水检测异常(cv::Exception): %s | code=%d | func=%s | "
                "file=%s:%d, 继续循环",
                e.what(), e.code, e.func.c_str(), e.file.c_str(), e.line);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_logger(), "[ir] 渗水检测异常: %s, 继续循环", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "[ir] 渗水检测未知异常, 继续循环");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    RCLCPP_INFO(get_logger(), "[ir] 红外渗水检测线程退出");
}

// ============================== 绘制工具 ==============================

void CoreNode::DrawPaneHeader(
    cv::Mat & pane_img, const std::string & title, const std::string & info)
{
    if (pane_img.empty())
    {
        return;
    }
    // 白底信息条(两行, 大字号): 第一行窗格标题, 第二行 src/det 帧率等
    cv::rectangle(
        pane_img, cv::Rect(0, 0, pane_img.cols, kPaneHeaderHeight),
        cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(
        pane_img, title, cv::Point(8, 22),
        cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
    cv::putText(
        pane_img, info, cv::Point(8, 46),
        cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(0, 110, 0), 1, cv::LINE_AA);
}

void CoreNode::DrawSensorOverlay(cv::Mat & canvas)
{
    // 文本内容(in-image 只用 ASCII: OpenCV Hershey 字体不支持中文)。
    // 传感器字段全部打印: T/RH/CO2 三值 + 温湿度/CO2 逐字段有效位 + 数据年龄。
    // 数据陈旧(超过 stale_timeout_sec_, 等同离线)时不再展示旧值。
    std::string line1;
    std::string line2;
    cv::Scalar color;
    // 锁内一次性快照: 数据 + 有效位 + 年龄(避免与 EnvCallback 竞争)
    sensor_interfaces::msg::EnvData env;
    bool fresh = false;
    double age = 0.0;
    {
        std::lock_guard<std::mutex> lock(env_mutex_);
        age = has_env_ ?
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - env_arrival_).count() :
            1e9;
        fresh = has_env_ && age < stale_timeout_sec_;   // 陈旧等同离线
        env = latest_env_;
    }
    if (!fresh)
    {
        line1 = "SENSOR: NO DATA (offline)";
        line2 = cv::format("waiting for /sensor/env ...");
        color = cv::Scalar(80, 80, 255);   // 红(OpenCV BGR)
    }
    else
    {
        line1 = cv::format(
            "SENSOR T=%.1fC RH=%.1f%% CO2=%.0fppm",
            env.temperature, env.humidity, env.co2);
        // 逐字段有效位全量打印(比合并词 VALID/PARTIAL 更直观):
        // 哪个字段失效一眼可见, 两个都失效等同离线(旧值不再展示)。
        line2 = cv::format("TH=%s CO2=%s age=%.1fs",
                           env.temp_hum_valid ? "ok" : "FAIL",
                           env.co2_valid ? "ok" : "FAIL", age);
        color = (env.temp_hum_valid && env.co2_valid) ?
            cv::Scalar(120, 255, 120) :          // 绿: 全部有效
            cv::Scalar(120, 200, 255);           // 橙: 部分字段失效
    }

    // 半透明黑底: 提高任何背景下的可读性。
    // y 从 60 起(避开窗格顶部 52px 信息条), 大字号全屏可读。
    const cv::Rect bg(6, 60, 600, 150);
    cv::Mat roi = canvas(bg);
    cv::Mat dark(roi.size(), roi.type(), cv::Scalar(20, 20, 20));
    cv::addWeighted(dark, 0.55, roi, 0.45, 0.0, roi);
    cv::rectangle(canvas, bg, color, 1, cv::LINE_AA);

    cv::putText(canvas, line1, cv::Point(16, 112),
                cv::FONT_HERSHEY_SIMPLEX, 0.88, color, 1, cv::LINE_AA);
    cv::putText(canvas, line2, cv::Point(16, 162),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, color, 1, cv::LINE_AA);
}

void CoreNode::DrawNavOverlay(cv::Mat & canvas)
{
    // 取协议快照(原子读)
    const uint8_t cmd = nav_cmd_.load(std::memory_order_relaxed);
    const uint8_t st = nav_status_.load(std::memory_order_relaxed);

    // 状态词(in-image 只用 ASCII): WAIT=等待指令, CAP=取图中,
    // DONE=取图完成, ACK?=等待导航确认(0x02 已发、导航未回 0x00)
    const char * state = "WAIT CMD";
    cv::Scalar color(120, 255, 120);          // 绿
    if (st == 0x02)
    {
        state = "DONE (wait nav 0x00)";
        color = cv::Scalar(120, 200, 255);    // 橙: 等待导航确认
    }
    else if (st == 0x01 || cmd == 0x01)
    {
        state = "CAPTURING";
        color = cv::Scalar(60, 220, 220);     // 黄: 取图中
    }

    // 半透明黑底(传感器块下方): 三行大字 —— 收到的指令 / 发出的状态 / 状态词
    const cv::Rect bg(6, 224, 600, 200);
    cv::Mat roi = canvas(bg);
    cv::Mat dark(roi.size(), roi.type(), cv::Scalar(20, 20, 20));
    cv::addWeighted(dark, 0.55, roi, 0.45, 0.0, roi);
    cv::rectangle(canvas, bg, color, 1, cv::LINE_AA);

    char l_nav[64];
    std::snprintf(l_nav, sizeof(l_nav), "NAV->VIS  0x%02X", cmd);
    char l_vis[64];
    std::snprintf(l_vis, sizeof(l_vis), "VIS->NAV  0x%02X", st);
    cv::putText(canvas, l_nav, cv::Point(16, 268),
                cv::FONT_HERSHEY_SIMPLEX, 0.88, color, 1, cv::LINE_AA);
    cv::putText(canvas, l_vis, cv::Point(16, 322),
                cv::FONT_HERSHEY_SIMPLEX, 0.88, color, 1, cv::LINE_AA);
    cv::putText(canvas, state, cv::Point(16, 384),
                cv::FONT_HERSHEY_SIMPLEX, 0.72, color, 1, cv::LINE_AA);
}

// ============================== 同屏显示线程 ==============================

void CoreNode::DisplayLoop()
{
    RCLCPP_INFO(get_logger(), "同屏显示线程运行中(窗口 %s)", kWindowName);

    // 窗口与 GUI 属性只在显示线程创建/设置(OpenCV GUI 非线程安全)。
    if (show_img_)
    {
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        if (fullscreen_)
        {
            // 无边框占满整屏; 画布与屏幕同比, 铺满无黑边
            cv::setWindowProperty(
                kWindowName, cv::WND_PROP_FULLSCREEN,
                cv::WINDOW_FULLSCREEN);
        }
    }

    const auto frame_interval =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                1.0 / std::max(display_fps_, 1.0)));

    auto next_tick = std::chrono::steady_clock::now();
    double display_fps_meas = 0.0;
    auto prev_tick = next_tick;

    while (rclcpp::ok() && display_run_)
    {
        // 定节拍刷新(上限 display_fps_), 不空转烧 CPU
        next_tick += frame_interval;
        std::this_thread::sleep_until(next_tick);
        if (std::chrono::steady_clock::now() > next_tick + std::chrono::seconds(1))
        {
            next_tick = std::chrono::steady_clock::now();   // 落后太多则重置
        }

        try
        {
            // 显示实际刷新率
            const auto now_t = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now_t - prev_tick).count();
            prev_tick = now_t;
            if (dt > 0.0)
            {
                display_fps_meas =
                    (display_fps_meas > 0.0) ?
                    (0.8 * display_fps_meas + 0.2 * (1.0 / dt)) : (1.0 / dt);
            }

            const int canvas_w = pane_width_ * 3 + pane_gap_ * 2;
            const cv::Size canvas_size(canvas_w, pane_height_);
            cv::Mat canvas(canvas_size, CV_8UC3, cv::Scalar(40, 40, 40));

            // ---- 三个窗格: 左 | 右 | 红外 ----
            const std::array<std::pair<StreamContext *, PaneContext *>, 3> panes =
            {{
                {&left_, &pane_left_},
                {&right_, &pane_right_},
                {&ir_, &pane_ir_},
            }};

            for (size_t i = 0; i < panes.size(); ++i)
            {
                StreamContext & stream = *panes[i].first;
                PaneContext & pane = *panes[i].second;
                const int x0 = static_cast<int>(i) * (pane_width_ + pane_gap_);

                // 取窗格标注结果(锁内拷贝)
                cv::Mat pane_img;
                uint64_t pane_seq = 0;
                {
                    std::lock_guard<std::mutex> lock(pane.mutex);
                    if (!pane.annotated.empty())
                    {
                        pane.annotated.copyTo(pane_img);
                        pane_seq = pane.seq;
                    }
                }

                // 离线判定: 该路超过 stale_timeout_sec_ 无新帧 -> 占位提示
                std::chrono::steady_clock::time_point arrival{};
                uint64_t stream_seq = 0;
                {
                    std::lock_guard<std::mutex> lock(stream.mutex);
                    arrival = stream.latest_arrival;
                    stream_seq = stream.seq;
                }
                const double since_frame =
                    (stream_seq == 0) ? 1e9 :
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - arrival).count();
                const bool offline = since_frame > stale_timeout_sec_;

                if (!offline && !pane_img.empty())
                {
                    pane_img.copyTo(canvas(cv::Rect(x0, 0, pane_width_, pane_height_)));
                }
                else
                {
                    // 占位: 深灰底 + 离线提示(热插拔等待接入时的画面)
                    cv::Mat cell = canvas(cv::Rect(x0, 0, pane_width_, pane_height_));
                    cell.setTo(cv::Scalar(28, 28, 28));
                    const std::string tip = offline ?
                        stream.title + "  OFFLINE" :
                        stream.title + "  decoding...";
                    const std::string tip2 =
                        "waiting for frames ...";
                    cv::putText(cell, tip, cv::Point(pane_width_ / 2 - 130, pane_height_ / 2 - 8),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(60, 60, 240), 2, cv::LINE_AA);
                    cv::putText(cell, tip2, cv::Point(pane_width_ / 2 - 110, pane_height_ / 2 + 24),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
                }
            }

            // ---- 传感器数据标注 + 导航协议状态(总图左上区域) ----
            DrawSensorOverlay(canvas);
            DrawNavOverlay(canvas);

            // ---- 总图右下角: 显示帧率 ----
            const std::string fps_text =
                cv::format("display %.1f fps", display_fps_meas);
            const int fps_w = cv::getTextSize(
                fps_text, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, nullptr).width;
            cv::putText(canvas, fps_text,
                        cv::Point(canvas_w - fps_w - 8, pane_height_ - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

            // 缓存总图(供按 S 保存"缺陷回溯"快照)
            {
                std::lock_guard<std::mutex> lock(composite_mutex_);
                canvas.copyTo(last_composite_);
            }

            // GUI 操作只发生在这个线程: 卡顿不影响回调/推理
            if (show_img_)
            {
                cv::imshow(kWindowName, canvas);
                cv::waitKey(1);
            }
        }
        catch (const cv::Exception & e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "显示线程异常(cv::Exception): %s | code=%d | func=%s | "
                "file=%s:%d, 继续循环",
                e.what(), e.code, e.func.c_str(), e.file.c_str(), e.line);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(
                get_logger(), "显示线程异常(std::exception): %s, 继续循环", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "显示线程未知异常, 继续循环");
        }
    }

    if (show_img_)
    {
        cv::destroyAllWindows();
    }
    RCLCPP_INFO(get_logger(), "同屏显示线程退出");
}

// ============================== 传感器回调 ==============================

void CoreNode::EnvCallback(
    const sensor_interfaces::msg::EnvData::SharedPtr msg)
{
    // 只存最新值(锁内拷贝), 显示/状态线程随时读快照
    std::lock_guard<std::mutex> lock(env_mutex_);
    latest_env_ = *msg;
    has_env_ = true;
    env_arrival_ = std::chrono::steady_clock::now();

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "环境数据: 温度=%.1f℃ 湿度=%.1f%%RH CO2=%.0fppm (%s)",
        msg->temperature, msg->humidity, msg->co2,
        (msg->temp_hum_valid && msg->co2_valid) ? "有效" : "部分无效");
}

// ============================== 状态发布 ==============================

void CoreNode::PublishStatus()
{
    // 各路 hz = 本秒累计 - 上秒累计(原子计数, 无需加锁)
    const uint64_t left_hz =
        left_.count.load(std::memory_order_relaxed) -
        left_.count_prev.load(std::memory_order_relaxed);
    const uint64_t right_hz =
        right_.count.load(std::memory_order_relaxed) -
        right_.count_prev.load(std::memory_order_relaxed);
    const uint64_t ir_hz =
        ir_.count.load(std::memory_order_relaxed) -
        ir_.count_prev.load(std::memory_order_relaxed);
    left_.count_prev.store(left_.count.load(std::memory_order_relaxed));
    right_.count_prev.store(right_.count.load(std::memory_order_relaxed));
    ir_.count_prev.store(ir_.count.load(std::memory_order_relaxed));

    // 传感器摘要(数据陈旧超过 stale_timeout_sec_ 视同离线:
    // 传感器运行中断连时 /sensor/env 静默, 但旧值仍在缓存)
    std::string env_txt = "no_data";
    {
        std::lock_guard<std::mutex> lock(env_mutex_);
        const bool fresh =
            has_env_ &&
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - env_arrival_).count() <
                stale_timeout_sec_;
        if (fresh)
        {
            env_txt = cv::format(
                "T=%.1f,H=%.1f,CO2=%.0f,valid=%d%d",
                latest_env_.temperature, latest_env_.humidity,
                latest_env_.co2,
                latest_env_.temp_hum_valid ? 1 : 0,
                latest_env_.co2_valid ? 1 : 0);
        }
    }

    // 离线标志(热插拔状态一眼可见)
    auto age_of = [](StreamContext & s) -> double
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        if (s.seq == 0)
        {
            return 1e9;   // 从未收到帧: 视为离线(不能返回负数, 会被当成"新鲜")
        }
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - s.latest_arrival).count();
    };
    const std::string l_state =
        age_of(left_) < stale_timeout_sec_ ? "on" : "OFF";
    const std::string r_state =
        age_of(right_) < stale_timeout_sec_ ? "on" : "OFF";
    const std::string i_state =
        age_of(ir_) < stale_timeout_sec_ ? "on" : "OFF";

    const std::string status = cv::format(
        "left_frames=%llu left_hz=%llu(%s) "
        "right_frames=%llu right_hz=%llu(%s) "
        "ir_frames=%llu ir_hz=%llu(%s) env[%s]",
        static_cast<unsigned long long>(left_.count.load()),
        static_cast<unsigned long long>(left_hz), l_state.c_str(),
        static_cast<unsigned long long>(right_.count.load()),
        static_cast<unsigned long long>(right_hz), r_state.c_str(),
        static_cast<unsigned long long>(ir_.count.load()),
        static_cast<unsigned long long>(ir_hz), i_state.c_str(),
        env_txt.c_str());

    std_msgs::msg::String status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "状态: %s", status.c_str());
}

// ============================== 保存一帧 ==============================

void CoreNode::SaveCurrentFrame()
{
    // 总图(含画框/渗水轮廓/传感器标注) —— 缺陷回溯的关键快照
    cv::Mat composite;
    {
        std::lock_guard<std::mutex> lock(composite_mutex_);
        if (!last_composite_.empty())
        {
            last_composite_.copyTo(composite);
        }
    }
    bool ok_any = false;
    if (!composite.empty())
    {
        ok_any = WriteMatToJpeg("composite", composite);
    }

    // 三路最新原始帧(可能为空: 刚被检测线程取走/尚未到帧, 跳过即可)
    cv::Mat left_img, right_img, ir_img;
    {
        std::lock_guard<std::mutex> lock(left_.mutex);
        if (!left_.latest_bgr.empty())
        {
            left_.latest_bgr.copyTo(left_img);
        }
    }
    {
        std::lock_guard<std::mutex> lock(right_.mutex);
        if (!right_.latest_bgr.empty())
        {
            right_.latest_bgr.copyTo(right_img);
        }
    }
    {
        std::lock_guard<std::mutex> lock(ir_.mutex);
        if (!ir_.latest_bgr.empty())
        {
            ir_.latest_bgr.copyTo(ir_img);
        }
    }
    if (!left_img.empty())
    {
        ok_any |= WriteMatToJpeg("left", left_img);
    }
    if (!right_img.empty())
    {
        ok_any |= WriteMatToJpeg("right", right_img);
    }
    if (!ir_img.empty())
    {
        ok_any |= WriteMatToJpeg("ir", ir_img);
    }

    if (!ok_any)
    {
        RCLCPP_WARN(
            get_logger(),
            "保存失败: 总图与三路帧都为空(可能还没收到图像), 或目录 %s 不可写",
            save_dir_.c_str());
    }
}

bool CoreNode::WriteMatToJpeg(const std::string & tag, const cv::Mat & image)
{
    try
    {
        // 文件名: capture_YYYYmmdd_HHMMSS_mmm_<tag>.jpg(本地时间)
        const auto now_sys = std::chrono::system_clock::now();
        const auto now_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now_sys.time_since_epoch()) % 1000;
        const std::time_t now_sec = std::chrono::system_clock::to_time_t(now_sys);
        char name[64];
        std::strftime(
            name, sizeof(name), "capture_%Y%m%d_%H%M%S", std::localtime(&now_sec));

        const std::string path =
            save_dir_ + "/" + name + "_" + tag + "_" +
            std::to_string(now_ms.count()) + ".jpg";

        std::error_code ec;
        std::filesystem::create_directories(save_dir_, ec);

        const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 90};
        std::vector<unsigned char> jpeg;
        if (!cv::imencode(".jpg", image, jpeg, encode_params))
        {
            RCLCPP_WARN(get_logger(), "%s 帧 JPEG 编码失败", tag.c_str());
            return false;
        }

        std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
        if (!ofs)
        {
            RCLCPP_WARN(
                get_logger(), "无法打开文件 %s 写入(errno=%d: %s)",
                path.c_str(), errno, std::strerror(errno));
            return false;
        }
        ofs.write(
            reinterpret_cast<const char *>(jpeg.data()),
            static_cast<std::streamsize>(jpeg.size()));
        ofs.close();

        RCLCPP_INFO(
            get_logger(), "已保存 %s 帧 -> %s (%zu 字节)",
            tag.c_str(), path.c_str(), jpeg.size());
        return true;
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            get_logger(), "保存 %s 帧异常: %s", tag.c_str(), e.what());
        return false;
    }
}

void CoreNode::KeyListenLoop()
{
    // 终端原始模式: VMIN=0/VTIME=2 -> read 最多阻塞 200ms,
    // 析构时置 key_run_=false 后最多 200ms 退出, join 不挂死。
    struct termios old_tio, raw_tio;
    ::tcgetattr(STDIN_FILENO, &old_tio);
    raw_tio = old_tio;
    raw_tio.c_lflag &= ~(ICANON | ECHO);
    raw_tio.c_cc[VMIN] = 0;
    raw_tio.c_cc[VTIME] = 2;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw_tio);

    char c;
    while (key_run_)
    {
        const ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n == 1 && (c == 's' || c == 'S'))
        {
            RCLCPP_INFO(get_logger(), "检测到 S 键, 保存总图+三路帧...");
            SaveCurrentFrame();
        }
        else if (n < 0 && errno != EINTR)
        {
            break;   // stdin 关闭
        }
    }

    // 恢复终端设置(否则退出后终端不回显)
    ::tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

// ============================== 析构 ==============================

CoreNode::~CoreNode()
{
    // 停止顺序: 先置标志 -> 逐个 join(线程内循环条件含 rclcpp::ok(),
    // Ctrl+C 后会自然退出; 标志位保证 shutdown 前也能退出)。
    detect_run_ = false;
    display_run_ = false;
    key_run_ = false;

    if (left_detect_thread_.joinable())
    {
        left_detect_thread_.join();
    }
    if (right_detect_thread_.joinable())
    {
        right_detect_thread_.join();
    }
    if (ir_detect_thread_.joinable())
    {
        ir_detect_thread_.join();
    }
    if (display_thread_.joinable())
    {
        display_thread_.join();
    }
    if (key_thread_.joinable())
    {
        key_thread_.join();
    }
}

}  // namespace culvert_core

// ============================== 主函数 ==============================
// MultiThreadedExecutor: 三路图像回调(独立回调组)并发执行,
// 单线程 spin 会让三路解码互相排队, 与多线程处理要求不符。

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<culvert_core::CoreNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
