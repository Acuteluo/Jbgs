// Copyright (c) 2024, galaxy_camera_dual contributors
//
// 文件: galaxy_camera_node.cpp
// 作用: 多相机驱动节点 GalaxyCameraNode 的实现(本包的核心)。
//
// ================================================================
// 与原包(单相机版 rm_vision_ros2_galaxy_camera)相比的改动总览
// ================================================================
//  1. 相机寻址: 每台相机支持 序列号/UserID/IP/设备序号(从 1 开始)
//     四种方式, 不再硬编码"打开 1 号相机";
//  2. 多相机: 一个节点管理 N 台相机(默认 left/right 两台), 每台相机:
//     独立采集线程、独立话题、独立 frame_id、独立标定文件;
//  3. 线程安全: 所有 SDK 调用经 GalaxyDevice 逐设备加锁; GXInitLib/
//     GXCloseLib 引用计数管理, 多节点/容器组合也安全;
//  4. 断线自愈: 某台相机连续超时或 SDK 掉线错误后, 由其采集线程自动
//     停采 -> 关设备 -> 重开 -> 恢复参数 -> 重采; 传输残帧只丢弃,
//     不会误触发重连风暴;
//  5. 时间戳: 默认用相机硬件 tick 打时间戳(低抖动), 可关闭;
//  6. 修复原包参数回调里"成功却标记失败"的逻辑写反的问题;
//  7. 修复原包 GXInitLib 失败时直接 exit() 拖死进程的问题,
//     改为循环重试并可用 Ctrl+C 优雅退出。
//
// ================================================================
// 线程模型
// ================================================================
//  galaxy_camera_dual_node(单进程)
//  |-- MultiThreadedExecutor   : 参数回调/服务(执行器线程)
//  |-- 采集线程 #1 (left)  ──┐
//  |    GXGetImage(阻塞) -> Bayer 转 RGB -> 发布
//  `-- 采集线程 #2 (right) ──┘
//  每台 GalaxyDevice 一把互斥锁; 关闭时: running=false -> 停采唤醒
//  -> join -> 关设备 -> 关库。
//
// ================================================================
// 参数说明
// ================================================================
//  全局参数:
//    camera_names                   相机组名列表, 如 [left, right]
//    sim_mode                       模拟模式(无实体相机时用模拟设备,
//                                   插拔语义与真机一致, 全链路可测)
//    sim.initial_absent_sec         模拟: 启动后设备离线时长(秒)
//    sim.disconnect_at_sec          模拟: 首次"中途拔掉"时刻(秒), <=0 关闭
//    sim.disconnect_duration_sec    模拟: 每次中途离线时长(秒)
//    sim.disconnect_repeat_period_sec 模拟: >0 时每隔该秒数重复离线(极端测试)
//    sim.image_dir                  模拟: 轮播实拍帧目录(空则生成合成图)
//    sim.width / sim.height         模拟: 帧分辨率
//    use_sensor_data_qos            是否使用 sensor_data QoS
//    use_camera_timestamp           是否用相机硬件 tick 打时间戳
//    timestamp_drift_compensation   是否补偿相机晶振与主机时钟的漂移
//    timestamp_exposure_center      是否把时间戳从曝光开始修正到曝光中心
//    grab_timeout_ms                GXGetImage 超时(毫秒)
//    publish_camera_info            是否发布 camera_info 标定话题
//                                   (默认关: 精简话题, 每台相机只发一条图像话题)
//
//  每相机参数(组名 = camera_names 里的名字, 如 left):
//    cameras.<name>.serial_number   按序列号打开(推荐, 最高优先级)
//    cameras.<name>.user_id         按 UserID 打开
//    cameras.<name>.ip_address      按 IP 打开
//    cameras.<name>.device_index    按序号打开(从 1 开始; 0=不用)
//    cameras.<name>.user_id_to_set  非空则烧写为相机持久 UserID
//    cameras.<name>.camera_name     话题/camera_info 名, 如 left_camera
//    cameras.<name>.frame_id        图像 frame_id
//    cameras.<name>.camera_info_url 标定文件 URL
//    cameras.<name>.exposure_time   曝光(us), 可在线修改
//    cameras.<name>.gain            增益, 可在线修改
//    cameras.<name>.frame_rate      帧率(Hz), 0=保持相机默认
//    cameras.<name>.packet_size     GigE GVSP 包长(主机 MTU=9000 推荐 8192)
//    cameras.<name>.throughput_limit_bps 设备端链路限速(B/s), 0=关闭
//    cameras.<name>.packet_delay    老固件限速兜底的 GVSP 包间隔(tick)
//    cameras.<name>.publish_compressed 是否额外发布 JPEG 压缩话题(给 rqt 流畅看图)
//    cameras.<name>.jpeg_quality    JPEG 质量 1-100(越小体积越小)

#include "galaxy_camera_dual/galaxy_camera_node.hpp"

#include "DxImageProc.h"                        // Bayer 转 RGB 算法
#include "galaxy_camera_dual/galaxy_device.hpp" // SDK 线程安全封装
#include "galaxy_camera_dual/simulated_galaxy_device.hpp"  // 模拟设备(sim_mode)

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/// 判断 SDK 状态码是否成功(0 为成功, 其余为错误码)。
#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

namespace galaxy_camera_dual
{

// ----------------------------------------------------------------------
// 文件内常量与工具函数(仅本文件可见)
// ----------------------------------------------------------------------
namespace
{
/// 相对路径解析: 以 JBGS_ROOT 环境变量(run.sh 导出)为基准,
/// 未设置时退化为进程 cwd; 绝对路径原样返回(用于 sim 轮播目录)。
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

/// 连续取图失败多少次后触发自动重连。
constexpr int kMaxFailCount = 5;

/// 连续取图超时多少次视为"无帧"(触发重连)。
/// 300ms 超时 × 10 次 ≈ 3 秒收不到任何帧 -> 判定相机掉线。
/// 不能太小: 帧率设置很低(如 1Hz)时偶发超时属正常。
constexpr int kMaxTimeoutCount = 10;

/// GXGetImage 默认超时(毫秒)。
/// 取值要点: 明显大于一帧周期(14fps 约 71ms), 又足够小,
/// 保证关机时 stopAcquisition 最多阻塞这一时长。
constexpr int kDefaultGrabTimeoutMs = 300;

/// 把 SDK 的像素格式枚举映射为 DxImageProc 使用的 Bayer 排列类型。
/// 相机输出 Bayer GR/RG/GB/BG 四种排列之一, 转 RGB 时必须匹配,
/// 否则颜色通道会错乱。
DX_PIXEL_COLOR_FILTER bayerTypeFromPixelFormat(int64_t pixel_format, bool * ok)
{
    *ok = true;
    switch (pixel_format)
    {
        case GX_PIXEL_FORMAT_BAYER_GR8:
            return BAYERGR;
        case GX_PIXEL_FORMAT_BAYER_RG8:
            return BAYERRG;
        case GX_PIXEL_FORMAT_BAYER_GB8:
            return BAYERGB;
        case GX_PIXEL_FORMAT_BAYER_BG8:
            return BAYERBG;
        default:
            *ok = false;
            return BAYERGR;
    }
}
}  // namespace

// ----------------------------------------------------------------------
// 状态码 -> 人话 (让日志不需要查手册就能看懂)
// ----------------------------------------------------------------------
inline const char * gxStatusName(GX_STATUS s)
{
    switch (s)
    {
    case GX_STATUS_SUCCESS: return "成功";
    case GX_STATUS_ERROR: return "SDK内部错误";
    case GX_STATUS_NOT_FOUND_TL: return "找不到传输层库";
    case GX_STATUS_NOT_FOUND_DEVICE: return "找不到设备";
    case GX_STATUS_OFFLINE: return "设备掉线";
    case GX_STATUS_INVALID_PARAMETER: return "参数无效";
    case GX_STATUS_INVALID_HANDLE: return "句柄无效";
    case GX_STATUS_INVALID_CALL: return "调用顺序错误";
    case GX_STATUS_INVALID_ACCESS: return "设备不可访问";
    case GX_STATUS_NEED_MORE_BUFFER: return "缓冲区不足";
    case GX_STATUS_ERROR_TYPE: return "功能码类型错误";
    case GX_STATUS_OUT_OF_RANGE: return "写入值越界";
    case GX_STATUS_NOT_IMPLEMENTED: return "功能未实现";
    case GX_STATUS_NOT_INIT_API: return "SDK未初始化";
    case GX_STATUS_TIMEOUT: return "超时无帧";
    default: return "未知状态码";
    }
}

inline const char * frameStatusName(int32_t s)
{
    switch (s)
    {
    case GX_FRAME_STATUS_SUCCESS: return "完整帧";
    case GX_FRAME_STATUS_INCOMPLETE: return "残帧(传输丢包)";
    default: return "未知帧状态";
    }
}

// ----------------------------------------------------------------------
// 单台相机的运行时上下文
// ----------------------------------------------------------------------
// 每台相机一个对象: 保存它的参数、SDK 封装、话题发布器、采集线程
// 以及时间戳换算所需的状态。采集线程只读写本对象的字段, 参数回调
// 线程只改写 exposure_time/gain, 二者互不重叠, 无需再加锁;
// exposure_time 例外: 时间戳修正会从采集线程读取它, 因此用原子类型;
// running 是原子变量, 用于"主线程要求采集线程退出"。
struct CameraContext
{
    std::string name;             ///< 参数组名, 如 "left"
    std::string camera_name;      ///< 话题/camera_info 名, 如 "left_camera"
    std::string frame_id;         ///< 图像 frame_id, 如 "left_camera_optical_frame"
    std::string camera_info_url;  ///< 标定文件 URL
    DeviceAddress address;        ///< 相机寻址方式(SN/UserID/IP/序号)
    std::string user_id_to_set;   ///< 非空则烧写为相机持久 UserID
    int64_t packet_size = 0;      ///< GigE 包长(字节); >0 则开流前设置(巨型帧防丢包)
    int64_t throughput_limit_bps = 0;  ///< 单相机链路带宽上限(B/s); 0 = 不限制
    int64_t packet_delay = 0;     ///< 吞吐限速不受支持时使用的 GVSP 包间隔(tick)
    // 原子类型: 采集线程(时间戳修正)与参数回调线程会跨线程读写。
    std::atomic<double> exposure_time{5000.0};  ///< 曝光时间(us)
    double gain = 1.0;              ///< 增益
    double frame_rate = 0.0;        ///< 帧率(Hz), 0 = 保持默认

    std::shared_ptr<GalaxyDevice> device;   ///< SDK 线程安全封装(每台一个)
    std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;  ///< 本机标定信息

    // ---- raw 无损话题(默认关闭: 15MB/帧是发布环节的主要开销) ----
    bool publish_raw = false;             ///< true 才发布 <camera_name>/image_raw
    image_transport::CameraPublisher raw_publisher;   ///< raw + 配套 camera_info
    // raw 关闭时, 标定信息由独立发布器发出(检测侧仍能拿到 camera_info)
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

    // ---- JPEG 压缩话题(检测链路与 rqt 共用; raw 默认关闭) ----
    bool publish_compressed = true;   ///< 是否发布 <camera_name>/image_raw/compressed
    int jpeg_quality = 75;            ///< JPEG 质量(1-100)
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub;
    std::vector<unsigned char> jpeg_buffer;   ///< JPEG 编码输出缓冲(复用)

    std::atomic<bool> running{false};   ///< 采集线程运行标志(原子)
    std::thread thread;                 ///< 本相机的采集线程
    std::thread trigger_thread;         ///< 软触发线程(frame_rate>0 时启用)
    bool opened = false;                ///< 是否成功打开(缺席相机跳过启动)
    int fail_count = 0;                 ///< 连续取图失败计数(触发重连用)
    int timeout_count = 0;              ///< 连续取图超时计数(相机掉线也会超时)

    // ---- 诊断统计(仅采集线程读写; 周期性打印健康报告) ----
    uint64_t stat_ok = 0;           ///< 完整帧计数
    uint64_t stat_timeout = 0;      ///< 超时(无帧到达)计数
    uint64_t stat_badframe = 0;     ///< 残帧(帧到达但数据不完整=丢包)计数
    uint64_t stat_error = 0;        ///< 其他错误计数
    uint64_t stat_since_ok = 0;     ///< 距上一完整帧的失败计数(报告用)
    std::chrono::steady_clock::time_point next_report =
        std::chrono::steady_clock::now() + std::chrono::seconds(30);

    // ---- 相机硬件 tick -> ROS 时间的换算状态 ----
    bool tick_freq_valid = false;   ///< 是否成功读到 tick 频率
    double tick_freq_hz = 1.0;      ///< tick 频率(Hz)
    bool time_base_valid = false;   ///< 是否已建立时间基准
    uint64_t base_ticks = 0;        ///< 基准帧的 tick 值
    rclcpp::Time base_time;         ///< 基准帧对应的 ROS 时刻
    // ---- 时钟漂移补偿状态(EMA 一阶低通) ----
    bool offset_valid = false;      ///< 偏差估计是否已初始化
    double offset_ns = 0.0;         ///< "相机侧时刻 -> 主机时刻"的平滑偏差(纳秒)
};

/// 节点的私有实现(Pimpl): 与头文件解耦, 内部类型不外泄。
struct GalaxyCameraNode::Impl
{
    bool use_sensor_data_qos = true;    ///< 是否用 sensor_data QoS
    bool use_camera_timestamp = true;   ///< 是否用相机硬件 tick 打时间戳
    bool timestamp_drift_compensation = true;  ///< 是否补偿时钟漂移(EMA)
    bool timestamp_exposure_center = false;    ///< 是否修正到曝光中心
    int grab_timeout_ms = kDefaultGrabTimeoutMs;  ///< 取图超时(毫秒)
    bool publish_camera_info = false;   ///< 是否发布 camera_info(精简话题默认关)
    // ---- 模拟模式(无实体相机时全链路可测, 插拔语义与真机一致) ----
    bool sim_mode = false;              ///< true = 用 SimulatedGalaxyDevice
    SimDeviceParams sim_params;         ///< 模拟设备的插拔/出图参数(全局共享)
    std::vector<std::shared_ptr<CameraContext>> cameras;  ///< 所有相机上下文
    std::atomic<bool> shutting{false};  ///< 全局停机标志(优雅关机用)
    std::thread hotplug_thread;         ///< 热插拔守望线程(缺席相机补开)
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle;
};

// ----------------------------------------------------------------------
// 构造函数: 按顺序完成六步初始化
// ----------------------------------------------------------------------
GalaxyCameraNode::GalaxyCameraNode(const rclcpp::NodeOptions & options)
: Node("galaxy_camera_dual", options),
  impl_(new Impl())
{
    RCLCPP_INFO(get_logger(), "Starting GalaxyCameraNode");

    // --------------------------------------------------------------
    // 第 1 步: 声明参数并读取全局配置(先行, 决定后续初始化路径)。
    // --------------------------------------------------------------
    impl_->use_sensor_data_qos = declare_parameter("use_sensor_data_qos", true);
    impl_->use_camera_timestamp = declare_parameter("use_camera_timestamp", true);
    impl_->timestamp_drift_compensation =
        declare_parameter("timestamp_drift_compensation", true);
    impl_->timestamp_exposure_center =
        declare_parameter("timestamp_exposure_center", false);
    impl_->grab_timeout_ms = declare_parameter(
        "grab_timeout_ms", static_cast<int>(kDefaultGrabTimeoutMs));
    impl_->publish_camera_info = declare_parameter("publish_camera_info", false);

    // ---- 模拟模式参数(launch 可用 sim:=true/false 覆盖) ----
    impl_->sim_mode = declare_parameter("sim_mode", false);
    impl_->sim_params.initial_absent_sec =
        declare_parameter("sim.initial_absent_sec", 8.0);
    impl_->sim_params.disconnect_at_sec =
        declare_parameter("sim.disconnect_at_sec", 30.0);
    impl_->sim_params.disconnect_duration_sec =
        declare_parameter("sim.disconnect_duration_sec", 8.0);
    impl_->sim_params.disconnect_repeat_period_sec =
        declare_parameter("sim.disconnect_repeat_period_sec", 0.0);
    impl_->sim_params.image_dir =
        ResolvePath(declare_parameter("sim.image_dir", ""));   // 空 = 程序合成图
    const int sim_width = declare_parameter("sim.width", 1280);
    const int sim_height = declare_parameter("sim.height", 960);
    impl_->sim_params.sim_width = std::max(64, sim_width);
    impl_->sim_params.sim_height = std::max(48, sim_height);
    if (impl_->sim_mode)
    {
        RCLCPP_INFO(
            get_logger(),
            "★ 模拟模式: 无实体相机, 使用 SimulatedGalaxyDevice "
            "(初始离线 %.1fs, 中途断连 %.1fs 起 / %.1fs 长 / 重复周期 %.1fs, "
            "轮播目录 %s)",
            impl_->sim_params.initial_absent_sec,
            impl_->sim_params.disconnect_at_sec,
            impl_->sim_params.disconnect_duration_sec,
            impl_->sim_params.disconnect_repeat_period_sec,
            impl_->sim_params.image_dir.c_str());
    }

    // --------------------------------------------------------------
    // 第 2 步: 初始化 GxIAPI 库(引用计数, 可重入)。
    // 失败时循环重试而不是 exit(): 可能只是相机还没上电/网卡没起来。
    // 期间按 Ctrl+C 会通过 rclcpp::ok() 优雅退出。
    // 模拟模式完全跳过 SDK(不碰 GXInitLib, 与大恒驱动解耦)。
    // --------------------------------------------------------------
    if (!impl_->sim_mode)
    {
        while (rclcpp::ok() && !GalaxyDevice::initLibrary())
        {
            RCLCPP_ERROR(get_logger(), "GXInitLib failed, retrying in 1 s ...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (!rclcpp::ok())
        {
            RCLCPP_WARN(
                get_logger(), "Startup interrupted while initializing GxIAPI");
            return;
        }
        RCLCPP_INFO(get_logger(), "GxIAPI library initialized");
    }

    // 相机组名列表(默认左右两台)。yaml 里形如 camera_names: [left, right]。
    const std::vector<std::string> camera_names = declare_parameter(
        "camera_names", std::vector<std::string>{"left", "right"});
    if (camera_names.empty())
    {
        // 不抛异常: 组件 main 没有 try/catch, 抛异常会直接 std::terminate。
        RCLCPP_FATAL(get_logger(), "camera_names must not be empty, node idles");
        return;
    }

    // 为每个相机组声明一组参数, 名字形如 cameras.left.serial_number。
    for (const std::string & name : camera_names)
    {
        declare_parameter<std::string>("cameras." + name + ".serial_number", "");
        declare_parameter<std::string>("cameras." + name + ".user_id", "");
        declare_parameter<std::string>("cameras." + name + ".ip_address", "");
        declare_parameter<int>("cameras." + name + ".device_index", 0);
        declare_parameter<std::string>("cameras." + name + ".user_id_to_set", "");
        declare_parameter<std::string>(
            "cameras." + name + ".camera_name", name + "_camera");
        declare_parameter<std::string>(
            "cameras." + name + ".frame_id", name + "_camera_optical_frame");
        declare_parameter<std::string>(
            "cameras." + name + ".camera_info_url",
            "package://galaxy_camera_dual/config/camera_info.yaml");
        declare_parameter<double>("cameras." + name + ".exposure_time", 5000.0);
        declare_parameter<double>("cameras." + name + ".gain", 1.0);
        declare_parameter<double>("cameras." + name + ".frame_rate", 0.0);
        declare_parameter<int>("cameras." + name + ".packet_size", 0);
        declare_parameter<int>(
            "cameras." + name + ".throughput_limit_bps", 0);
        declare_parameter<int>("cameras." + name + ".packet_delay", 0);
        declare_parameter<bool>("cameras." + name + ".publish_compressed", true);
        declare_parameter<int>("cameras." + name + ".jpeg_quality", 75);
        declare_parameter<bool>("cameras." + name + ".publish_raw", false);
        // 模拟模式专用: 每台相机独立的轮播目录(左右可接入不同画面,
        // 非空时覆盖全局 sim.image_dir) —— 用于"左右不同场景"的仿真
        declare_parameter<std::string>("cameras." + name + ".sim_image_dir", "");
        // 模拟模式专用: 每台相机独立的插拔时间窗覆盖(热插拔测试需要
        // 单独控制某一台相机的初始缺失/中途断连)。
        // 哨兵值 -1.0 = 不覆盖(继承全局 sim.*); disconnect_at_sec 的
        // 覆盖值 0 表示"明确禁用中途断连"。
        declare_parameter<double>(
            "cameras." + name + ".sim.initial_absent_sec", -1.0);
        declare_parameter<double>(
            "cameras." + name + ".sim.disconnect_at_sec", -1.0);
        declare_parameter<double>(
            "cameras." + name + ".sim.disconnect_duration_sec", -1.0);
        declare_parameter<double>(
            "cameras." + name + ".sim.disconnect_repeat_period_sec", -1.0);
    }

    // --------------------------------------------------------------
    // 第 4 步: 枚举设备并逐台打印 SN/IP/MAC。
    // 目的: 让用户第一次就能确认哪台是"左"、哪台是"右",
    // 然后把序列号填进参数文件(按序号寻址重启后可能互换)。
    //
    // 热插拔要点(修复历史缺陷): 这里只做"有限次尝试"(约 3 秒),
    // 一台设备都没有时不再死循环阻塞 —— 否则节点永远走不到后面,
    // 热插拔守望线程也起不来, "初始未接入 -> 接上"就永远无法恢复。
    // 现在的行为: 枚举失败也继续启动, 全部相机保持"离线"状态,
    // 由 hotPlugLoop 每 3 秒探测, 设备上电/插网线后自动补开。
    // --------------------------------------------------------------
    if (!impl_->sim_mode)
    {
        bool listed = false;
        for (int attempt = 1; attempt <= 3 && rclcpp::ok(); ++attempt)
        {
            const std::vector<DeviceInfo> devices = GalaxyDevice::listDevices(1000);
            if (!devices.empty())
            {
                RCLCPP_INFO(get_logger(), "Found %zu device(s):", devices.size());
                for (const DeviceInfo & d : devices)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "  [%u] model=%s SN=%s UserID=%s IP=%s MAC=%s",
                        d.index, d.model_name.c_str(), d.serial_number.c_str(),
                        d.user_id.c_str(), d.ip.c_str(), d.mac.c_str());
                }
                listed = true;
                break;
            }
            RCLCPP_WARN(
                get_logger(),
                "No camera found (attempt %d/3); will keep waiting via "
                "hot-plug watcher ...", attempt);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        (void)listed;   // 枚举结果只用于日志, 不阻塞启动
        if (!rclcpp::ok())
        {
            RCLCPP_WARN(get_logger(), "Startup interrupted while enumerating devices");
            return;
        }
    }

    // --------------------------------------------------------------
    // 第 5 步: 逐台构建相机上下文并尝试打开。
    // 打开失败(设备未上电/被占用/模拟离线窗)不再放弃整台相机:
    //   - 话题发布器照常创建(热插拔恢复后立即可发布);
    //   - opened 保持 false, 由热插拔守望线程每 3 秒补开;
    //   - 单台相机缺席完全不影响其余相机启动(单相机也能跑)。
    // --------------------------------------------------------------
    for (size_t cam_idx = 0; cam_idx < camera_names.size(); ++cam_idx)
    {
        const std::string & name = camera_names[cam_idx];

        // 先把 yaml 参数读进本相机的上下文。
        auto ctx = std::make_shared<CameraContext>();
        ctx->name = name;
        ctx->address.serial_number =
            get_parameter("cameras." + name + ".serial_number").as_string();
        ctx->address.user_id =
            get_parameter("cameras." + name + ".user_id").as_string();
        ctx->address.ip_address =
            get_parameter("cameras." + name + ".ip_address").as_string();
        ctx->address.device_index = static_cast<uint32_t>(
            get_parameter("cameras." + name + ".device_index").as_int());
        ctx->user_id_to_set =
            get_parameter("cameras." + name + ".user_id_to_set").as_string();
        ctx->camera_name =
            get_parameter("cameras." + name + ".camera_name").as_string();
        ctx->frame_id =
            get_parameter("cameras." + name + ".frame_id").as_string();
        ctx->camera_info_url =
            get_parameter("cameras." + name + ".camera_info_url").as_string();
        ctx->exposure_time =
            get_parameter("cameras." + name + ".exposure_time").as_double();
        ctx->gain = get_parameter("cameras." + name + ".gain").as_double();
        ctx->frame_rate =
            get_parameter("cameras." + name + ".frame_rate").as_double();
        ctx->packet_size =
            get_parameter("cameras." + name + ".packet_size").as_int();
        ctx->throughput_limit_bps = get_parameter(
            "cameras." + name + ".throughput_limit_bps").as_int();
        ctx->packet_delay =
            get_parameter("cameras." + name + ".packet_delay").as_int();
        ctx->publish_compressed =
            get_parameter("cameras." + name + ".publish_compressed").as_bool();
        ctx->jpeg_quality =
            get_parameter("cameras." + name + ".jpeg_quality").as_int();
        ctx->publish_raw =
            get_parameter("cameras." + name + ".publish_raw").as_bool();

        // 创建设备对象: 模拟模式用 SimulatedGalaxyDevice(每台错开
        // 断连时刻, 便于观察多相机独立恢复), 真机用 GalaxyDevice。
        if (impl_->sim_mode)
        {
            SimDeviceParams p = impl_->sim_params;
            p.frame_rate = ctx->frame_rate;
            p.label = ctx->camera_name;
            // 每相机独立画面目录(左右接入不同场景): 非空覆盖全局配置
            const std::string per_cam_dir = ResolvePath(
                get_parameter("cameras." + name + ".sim_image_dir").as_string());
            if (!per_cam_dir.empty())
            {
                p.image_dir = per_cam_dir;
            }
            // 每相机独立的插拔时间窗覆盖(-1 = 继承全局)
            const double cam_absent = get_parameter(
                "cameras." + name + ".sim.initial_absent_sec").as_double();
            const double cam_disc = get_parameter(
                "cameras." + name + ".sim.disconnect_at_sec").as_double();
            const double cam_disc_dur = get_parameter(
                "cameras." + name + ".sim.disconnect_duration_sec").as_double();
            const double cam_disc_rep = get_parameter(
                "cameras." + name + ".sim.disconnect_repeat_period_sec").as_double();
            if (cam_absent >= 0.0)
            {
                p.initial_absent_sec = cam_absent;
            }
            if (cam_disc >= 0.0)
            {
                p.disconnect_at_sec = cam_disc;
            }
            if (cam_disc_dur >= 0.0)
            {
                p.disconnect_duration_sec = cam_disc_dur;
            }
            if (cam_disc_rep >= 0.0)
            {
                p.disconnect_repeat_period_sec = cam_disc_rep;
            }
            // 双相机中途断连错开 3 秒, 避免"同时断同时恢复"掩盖单台问题
            // (仅当该相机没有显式覆盖 disconnect_at 时才错开)
            if (cam_disc < 0.0 && p.disconnect_at_sec > 0.0)
            {
                p.disconnect_at_sec += 3.0 * static_cast<double>(cam_idx);
            }
            RCLCPP_INFO(
                get_logger(),
                "Camera '%s' 生效模拟窗: 初始缺失=%.2fs, 断连@%.2fs/%.2fs, "
                "重复=%.2fs (cam_disc=%.2f)",
                name.c_str(), p.initial_absent_sec, p.disconnect_at_sec,
                p.disconnect_duration_sec, p.disconnect_repeat_period_sec,
                cam_disc);
            ctx->device = std::make_shared<SimulatedGalaxyDevice>(name, p);
        }
        else
        {
            ctx->device = std::make_shared<GalaxyDevice>();
        }

        // 注意: 设备打开在第二阶段(所有设备对象先统一构造, 保证模拟
        // 时间窗的"设备时间零点"对齐; 打开重试可能阻塞数秒, 若放在此处
        // 会使后构造设备的 sim 时间窗整体后移)。

        // 加载本相机的标定文件(左右相机各一份)。
        // 注: 默认不再发布 camera_info 话题(精简话题), 文件仍加载,
        // 需要时把 publish_camera_info 打开即可恢复发布。
        ctx->info_manager =
            std::make_unique<camera_info_manager::CameraInfoManager>(
                this, ctx->camera_name);
        if (ctx->info_manager->validateURL(ctx->camera_info_url))
        {
            ctx->info_manager->loadCameraInfo(ctx->camera_info_url);
            ctx->camera_info_msg = ctx->info_manager->getCameraInfo();
        }
        else
        {
            RCLCPP_WARN(
                get_logger(), "Invalid camera info URL for '%s': %s",
                name.c_str(), ctx->camera_info_url.c_str());
        }

        const auto qos = impl_->use_sensor_data_qos ?
            rmw_qos_profile_sensor_data : rmw_qos_profile_default;

        // raw 话题默认关闭(15MB/帧是发布环节的主要开销, 且当前无人订阅)。
        // 需要无损时把 yaml 里 publish_raw 改成 true。
        if (ctx->publish_raw)
        {
            ctx->raw_publisher = image_transport::create_camera_publisher(
                this, ctx->camera_name + "/image_raw", qos);
        }
        else if (impl_->publish_camera_info)
        {
            // 精简模式下不发布; 打开 publish_camera_info 后, 标定信息由
            // 独立发布器发出(每帧都发, 体积极小, transient_local 立即送达)。
            const rclcpp::QoS info_qos = rclcpp::QoS(1)
                .reliable().transient_local();
            ctx->camera_info_pub =
                create_publisher<sensor_msgs::msg::CameraInfo>(
                    ctx->camera_name + "/camera_info", info_qos);
        }

        // JPEG 压缩话题: 命名遵循 image_transport 惯例
        // (<base>/compressed), rqt_image_view 的话题下拉框里会直接出现
        // "/<camera_name>/image_raw/compressed", 选它即可流畅看图。
        // ★ 这是本节点默认唯一的话题(话题精简: 每台相机一条)。
        if (ctx->publish_compressed)
        {
            // rclcpp 的发布器需要 rclcpp::QoS 类型(与 image_transport 不同)
            const rclcpp::QoS compressed_qos(
                rclcpp::QoSInitialization::from_rmw(qos), qos);
            ctx->compressed_pub =
                create_publisher<sensor_msgs::msg::CompressedImage>(
                    ctx->camera_name + "/image_raw/compressed", compressed_qos);
        }

        impl_->cameras.push_back(ctx);
    }

    // --------------------------------------------------------------
    // 第 5.5 步: 第二阶段 —— 逐台尝试打开设备(失败重试 3 次后标记离线,
    // 交给热插拔守望线程; 打开成功则读取 tick 频率并下发参数)。
    // 与第一阶段分离的原因见上(模拟时间零点对齐)。
    // --------------------------------------------------------------
    for (auto & ctx : impl_->cameras)
    {
        int open_attempts = 0;
        while (rclcpp::ok() && !ctx->device->isOpen())
        {
            std::string error;
            if (ctx->device->open(ctx->address, &error))
            {
                break;
            }
            ++open_attempts;
            if (open_attempts >= 3)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Camera '%s': open failed 3 times (%s), "
                    "标记为离线并保持等待(热插拔守望线程将每 3 秒重试; "
                    "设备上电/接线恢复后自动接入)",
                    ctx->name.c_str(), error.c_str());
                break;
            }
            RCLCPP_WARN(
                get_logger(),
                "Camera '%s' open failed (%s), retrying in 1 s ...",
                ctx->name.c_str(), error.c_str());
            if (error.find("-5") != std::string::npos)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "  [提示] 状态码 -5 常见原因: ①寻址参数为空(直接 ros2 run 时节点名与 yaml 命名空间不匹配, 请用 launch 或 -r __node:=galaxy_camera); ②相机正被另一个客户端独占占用"
                    "(原包节点 / 大恒客户端 / 本包的另一个实例)。"
                    "请先关闭占用方再启动本节点。");
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (!rclcpp::ok())
        {
            RCLCPP_WARN(get_logger(), "Startup interrupted while opening cameras");
            return;
        }

        // 只有打开成功才标记 opened(修复历史缺陷: 旧代码在失败路径后
        // 无条件置 true, 导致离线相机被误判为在线)。
        ctx->opened = ctx->device->isOpen();
        if (ctx->opened)
        {
            RCLCPP_INFO(
                get_logger(), "Camera '%s' opened: %s",
                ctx->name.c_str(), ctx->device->describe().c_str());

            // 读取相机硬件 tick 频率(部分型号不支持, 失败则退回 ROS 时间戳)。
            int64_t tick_freq = 0;
            if (ctx->device->getInt(GX_INT_TIMESTAMP_TICK_FREQUENCY, &tick_freq) &&
                tick_freq > 0)
            {
                ctx->tick_freq_hz = static_cast<double>(tick_freq);
                ctx->tick_freq_valid = true;
            }

            // 下发曝光/增益/帧率等设置(详见 applyCameraSettings)。
            applyCameraSettings(*ctx);
        }
    }

    // --------------------------------------------------------------
    // 第 6 步: 注册参数回调, 支持运行时在线调曝光/增益。
    // --------------------------------------------------------------
    impl_->params_callback_handle = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters)
        {
            return parametersCallback(parameters);
        });

    // --------------------------------------------------------------
    // 第 7 步: 每台"在线"相机启动一个独立采集线程;
    // 离线相机由热插拔守望线程(下一步)自动补开后再拉起线程。
    // --------------------------------------------------------------
    for (auto & ctx : impl_->cameras)
    {
        if (!ctx->opened)
        {
            RCLCPP_INFO(
                get_logger(),
                "Camera '%s' 当前离线, 等待热插拔守望线程自动接入",
                ctx->name.c_str());
            continue;
        }
        if (!ctx->device->startAcquisition())
        {
            RCLCPP_ERROR(
                get_logger(), "Camera '%s': failed to start acquisition",
                ctx->name.c_str());
        }
        ctx->running = true;
        ctx->thread = std::thread(
            [this, ctx]()
            {
                captureLoop(ctx);
            });
        if (ctx->frame_rate > 0.0)
        {
            const size_t idx = trigger_index_++;
            ctx->trigger_thread = std::thread(
                [this, ctx, idx]()
                {
                    triggerLoop(ctx, idx);
                });
        }
    }

    // 启动热插拔守望线程: 启动时缺席的相机后续上电/接线恢复后自动补开。
    impl_->hotplug_thread = std::thread(
        [this]()
        {
            hotPlugLoop();
        });

    RCLCPP_INFO(
        get_logger(), "GalaxyCameraNode initialized with %zu camera(s)",
        impl_->cameras.size());
}

// ----------------------------------------------------------------------
// 析构函数: 按固定顺序收尾, 保证无死锁、无残留线程
// ----------------------------------------------------------------------
GalaxyCameraNode::~GalaxyCameraNode()
{
    RCLCPP_INFO(get_logger(), "Shutting down GalaxyCameraNode ...");
    if (!impl_)
    {
        return;
    }

    // 顺序说明(不能调换):
    //  1) 先把每台相机的 running 置 false, 让采集线程退出主循环;
    //  2) 再停采: 若采集线程此刻正阻塞在 GXGetImage 里,
    //     停采命令会让它立刻返回(最多等一个取图超时);
    //  3) join 所有线程, 保证之后没有线程再碰设备;
    //  4) 最后才关设备、清列表、关库。

    // 1) 先通知热插拔守望线程退出(避免它和析构抢设备)。
    impl_->shutting = true;
    if (impl_->hotplug_thread.joinable())
    {
        impl_->hotplug_thread.join();
    }

    // 2) 通知所有采集/触发线程退出, 并停采唤醒阻塞中的取图。
    for (auto & ctx : impl_->cameras)
    {
        if (!ctx)
        {
            continue;
        }
        ctx->running = false;
        if (ctx->device)
        {
            ctx->device->stopAcquisition();
        }
    }
    for (auto & ctx : impl_->cameras)
    {
        if (ctx && ctx->thread.joinable())
        {
            ctx->thread.join();
        }
    }
    for (auto & ctx : impl_->cameras)
    {
        if (ctx && ctx->trigger_thread.joinable())
        {
            ctx->trigger_thread.join();
        }
    }
    for (auto & ctx : impl_->cameras)
    {
        if (ctx && ctx->device)
        {
            ctx->device->close();
        }
    }
    impl_->cameras.clear();

    // 引用计数关库: 本进程里若还有其它节点在用库, 这里不会误关。
    GalaxyDevice::closeLibrary();
    RCLCPP_INFO(get_logger(), "GalaxyCameraNode destroyed");
}

// ----------------------------------------------------------------------
// 热插拔守望线程: 每 3 秒尝试补开启动时缺席的相机。
//   - 相机上电/网线插回后自动恢复, 无需重启节点;
//   - 和析构配合: shutting=true 时立即退出, 不抢设备。
// ----------------------------------------------------------------------
void GalaxyCameraNode::hotPlugLoop()
{
    RCLCPP_INFO(get_logger(), "热插拔守望线程已启动");
    while (!impl_->shutting && rclcpp::ok())
    {
        for (auto & ctx : impl_->cameras)
        {
            if (!ctx || ctx->opened || impl_->shutting)
            {
                continue;
            }
            // 尝试打开缺席相机(单次, 不重试阻塞)
            std::string error;
            if (!ctx->device->open(ctx->address, &error))
            {
                // 仍不在线: 节流打印等待提示, 3 秒后再试
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 15000,
                    "热插拔: 相机 '%s' 仍未接入(%s), 继续等待 ...",
                    ctx->name.c_str(), error.c_str());
                continue;
            }
            RCLCPP_INFO(
                get_logger(), "热插拔: 相机 '%s' 已上线 (%s)",
                ctx->name.c_str(), ctx->device->describe().c_str());

            // 读取 tick 频率(与构造期路径一致; 模拟设备同样提供)
            int64_t tick_freq = 0;
            if (ctx->device->getInt(GX_INT_TIMESTAMP_TICK_FREQUENCY, &tick_freq) &&
                tick_freq > 0)
            {
                ctx->tick_freq_hz = static_cast<double>(tick_freq);
                ctx->tick_freq_valid = true;
            }

            // 下参数、启采集
            applyCameraSettings(*ctx);
            if (!ctx->device->startAcquisition())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "热插拔: 相机 '%s' 打开成功但启采集失败, 关闭后下次再试",
                    ctx->name.c_str());
                ctx->device->close();
                continue;
            }

            // 标记上线并拉起采集/触发线程
            ctx->opened = true;
            ctx->fail_count = 0;
            ctx->timeout_count = 0;
            ctx->running = true;
            ctx->thread = std::thread(
                [this, ctx]()
                {
                    captureLoop(ctx);
                });
            if (ctx->frame_rate > 0.0)
            {
                const size_t idx = trigger_index_++;
                ctx->trigger_thread = std::thread(
                    [this, ctx, idx]()
                    {
                        triggerLoop(ctx, idx);
                    });
            }
            RCLCPP_INFO(
                get_logger(), "热插拔: 相机 '%s' 已恢复运行",
                ctx->name.c_str());
        }
        // 分片睡眠, 可被 shutting 打断
        for (int i = 0; i < 30 && !impl_->shutting && rclcpp::ok(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    RCLCPP_INFO(get_logger(), "热插拔守望线程已退出");
}

// ----------------------------------------------------------------------
// 把参数下发给相机(打开后 / 重连后都会调用)
// ----------------------------------------------------------------------
void GalaxyCameraNode::applyCameraSettings(CameraContext & ctx)
{
    // GigE 健壮性设置:
    //  - 命令超时 1000ms: 网络抖动时给命令留足重试时间;
    //  - SDK 内部采集队列 5 帧: 消费速度有波动时不至于丢帧;
    //  - 开启 GVSP 丢包重传: 少量偶发丢包可在交付帧前被救回。
    ctx.device->setInt(GX_DEV_INT_COMMAND_TIMEOUT, 1000);
    ctx.device->setAcquisitionBufferNumber(5);
    if (!ctx.device->setEnum(GX_DS_ENUM_RESEND_MODE, GX_DS_RESEND_MODE_ON))
    {
        RCLCPP_WARN(
            get_logger(),
            "Camera '%s': failed to enable GVSP packet resend",
            ctx.name.c_str());
    }

    // 可选: 把配置的字符串烧写为相机持久 UserID(写一次即可长期使用,
    // 之后就能用 user_id: left/right 稳定寻址, 不依赖枚举顺序)。
    if (!ctx.user_id_to_set.empty())
    {
        if (ctx.device->setString(GX_STRING_DEVICE_USERID, ctx.user_id_to_set))
        {
            RCLCPP_INFO(
                get_logger(), "Camera '%s': UserID written as '%s' (persistent)",
                ctx.name.c_str(), ctx.user_id_to_set.c_str());
        }
        else
        {
            RCLCPP_WARN(
                get_logger(), "Camera '%s': failed to write UserID",
                ctx.name.c_str());
        }
    }

    // GigE 包长(巨型帧): 必须在开流前设置; 需与网卡 MTU(≥ 包长+包头)配合。
    // 作用: 5MP 帧拆成 8192 字节大包后包数量降为约 1/5, 从根上缓解
    // "小包突发 -> 内核/网卡缓冲溢出 -> 不完整帧(incomplete)" 的丢包链。
    if (ctx.packet_size > 0)
    {
        if (ctx.device->setInt(GX_INT_GEV_PACKETSIZE, ctx.packet_size))
        {
            RCLCPP_INFO(
                get_logger(),
                "Camera '%s': packet size set to %ld bytes (jumbo frames)",
                ctx.name.c_str(), static_cast<long>(ctx.packet_size));
        }
        else
        {
            // 多数为只读(由 SDK 按 MTU 自动协商): 读出实际值便于确认。
            int64_t actual = 0;
            if (ctx.device->getInt(GX_INT_GEV_PACKETSIZE, &actual))
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Camera '%s': packet size read-only, negotiated=%ld "
                    "(MTU=9000 时推荐 8192)",
                    ctx.name.c_str(), static_cast<long>(actual));
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Camera '%s': failed to set packet size %ld "
                    "(检查网卡 MTU 是否 >= %ld)",
                    ctx.name.c_str(), static_cast<long>(ctx.packet_size),
                    static_cast<long>(ctx.packet_size));
            }
        }
    }

    // 单网卡挂双 GigE 相机时，仅靠错相触发仍可能因为调度抖动让两个 5MP
    // 帧的 UDP 包突发重叠。设备端链路限速会把每帧均匀铺在周期内，避免
    // 瞬时超过接收网卡能力。10Hz/5,038,848B 每台理论最低约 50.4MB/s；
    // 双机共用千兆链路时建议每台 55MB/s，总量约 880Mbps，保留协议余量。
    bool throughput_configured = false;
    if (ctx.throughput_limit_bps > 0)
    {
        const bool supported =
            ctx.device->isImplemented(GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT) &&
            ctx.device->isImplemented(
                GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE);
        const bool mode_ok = supported && ctx.device->setEnum(
            GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE,
            GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ON);
        const bool limit_ok = supported && ctx.device->setInt(
            GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT,
            ctx.throughput_limit_bps);
        if (mode_ok && limit_ok)
        {
            throughput_configured = true;
            RCLCPP_INFO(
                get_logger(),
                "Camera '%s': link throughput limited to %.1f MB/s",
                ctx.name.c_str(),
                static_cast<double>(ctx.throughput_limit_bps) / 1000000.0);
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Camera '%s': device-side throughput limit %ld B/s is not "
                "supported or could not be set; continuing with staggered "
                "software triggers",
                ctx.name.c_str(),
                static_cast<long>(ctx.throughput_limit_bps));
        }
    }

    // 老款 MER 固件可能没有 DeviceLinkThroughputLimit，但该系列支持
    // GevSCPD/GX_INT_GEV_PACKETDELAY。仅当用户按具体固件实测后配置了
    // 非零值时才启用；tick 周期随固件而异，写死过大值会让单帧传输超过
    // 触发周期，造成隔次触发被忽略、10Hz 实际只剩约 5Hz。
    if (!throughput_configured && ctx.packet_delay > 0)
    {
        if (ctx.device->setInt(GX_INT_GEV_PACKETDELAY, ctx.packet_delay))
        {
            RCLCPP_INFO(
                get_logger(),
                "Camera '%s': GVSP packet delay set to %ld ticks "
                "(throughput-limit fallback)",
                ctx.name.c_str(), static_cast<long>(ctx.packet_delay));
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Camera '%s': failed to set GVSP packet delay %ld; use a "
                "separate NIC if incomplete frames persist",
                ctx.name.c_str(), static_cast<long>(ctx.packet_delay));
        }
    }

    // 曝光时间(us)。
    if (!ctx.device->setFloat(GX_FLOAT_EXPOSURE_TIME, ctx.exposure_time.load()))
    {
        RCLCPP_WARN(
            get_logger(), "Camera '%s': failed to set exposure_time",
            ctx.name.c_str());
    }

    // 增益。
    if (!ctx.device->setFloat(GX_FLOAT_GAIN, ctx.gain))
    {
        RCLCPP_WARN(
            get_logger(), "Camera '%s': failed to set gain",
            ctx.name.c_str());
    }

    // 帧率控制(frame_rate > 0): 采用"软触发"方案 ——
    //   打开触发模式并把触发源设为软件, 之后由节点的触发线程按目标
    //   帧率发 GX_COMMAND_TRIGGER_SOFTWARE。帧率完全由我们掌控,
    //   与相机固件是否支持"帧率开关"无关(实测左机固件不吃枚举开关)。
    // frame_rate == 0: 保持连续采集(触发模式关闭)。
    if (ctx.frame_rate > 0.0)
    {
        const bool mode_ok = ctx.device->setEnum(
            GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        const bool src_ok = ctx.device->setEnum(
            GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
        RCLCPP_INFO(
            get_logger(),
            "Camera '%s': 软触发模式 -> mode:%s source:%s, "
            "目标帧率 %.1f Hz 由触发线程控制",
            ctx.name.c_str(), mode_ok ? "OK" : "FAIL",
            src_ok ? "OK" : "FAIL", ctx.frame_rate);
    }
    else
    {
        if (!ctx.device->setEnum(GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF))
        {
            RCLCPP_WARN(
                get_logger(), "Camera '%s': failed to disable trigger mode",
                ctx.name.c_str());
        }
    }
}

// ----------------------------------------------------------------------
// 硬件 tick -> ROS 时间
// ----------------------------------------------------------------------
// 原理: 相机在"曝光开始(帧开始)"的瞬间把内部高精度计数器锁存进
// nTimestamp。换算公式: ROS时刻 = 基准ROS时刻 + (tick差值 / tick频率)。
// 帧间间隔完全由相机晶振决定, 抖动为微秒级。
//
// 两个可选修正:
//  1) 时钟漂移补偿(默认开): 相机晶振与主机时钟有 ppm 级频差, 长时间
//     运行会累积毫秒级偏差。用"帧到达时刻 - 相机侧时刻"的实测偏差做
//     一阶低通(EMA, alpha=0.02), 得到平滑的偏差估计叠加回时间戳,
//     既保留低抖动, 又让时间戳长期跟随主机时钟(与里程计/IMU对齐)。
//     网络延迟带来的抖动被 EMA 平均掉, 只有缓变的时钟漂移被跟踪。
//  2) 曝光中心修正(默认关): 全局快门相机整帧同时曝光, 运动补偿等
//     场景更希望用"曝光中心 = 曝光开始 + 曝光时间/2"作为时间戳。
rclcpp::Time GalaxyCameraNode::frameTimestamp(
    const std::shared_ptr<CameraContext> & ctx, uint64_t device_ticks)
{
    // 未启用硬件时间戳, 或相机不支持 tick 时, 退回 ROS 接收时刻。
    if (!impl_->use_camera_timestamp || !ctx->tick_freq_valid)
    {
        return now();
    }

    // 以第一帧为基准建立映射: (tick 差值 / tick 频率) = 相对秒数。
    if (!ctx->time_base_valid)
    {
        ctx->base_ticks = device_ticks;
        ctx->base_time = now();
        ctx->time_base_valid = true;
    }
    const double dt_ticks = static_cast<double>(device_ticks - ctx->base_ticks);
    const int64_t dt_ns =
        static_cast<int64_t>(dt_ticks * 1e9 / ctx->tick_freq_hz);
    rclcpp::Time stamp = ctx->base_time + rclcpp::Duration::from_nanoseconds(dt_ns);

    // 时钟漂移补偿: 实测偏差 = 到达时刻 - 相机侧时刻, EMA 平滑。
    if (impl_->timestamp_drift_compensation)
    {
        const int64_t measured_offset_ns =
            (now() - stamp).nanoseconds();
        if (!ctx->offset_valid)
        {
            // 第一帧直接取实测值, 避免冷启动跳变。
            ctx->offset_ns = static_cast<double>(measured_offset_ns);
            ctx->offset_valid = true;
        }
        else
        {
            // alpha 小 -> 平滑强: 抖动被平均, 漂移被跟踪。
            constexpr double kAlpha = 0.02;
            ctx->offset_ns =
                kAlpha * static_cast<double>(measured_offset_ns) +
                (1.0 - kAlpha) * ctx->offset_ns;
        }
        stamp = stamp + rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(ctx->offset_ns));
    }

    // 曝光中心修正: 曝光开始 + 曝光时间/2(曝光时间单位为 us)。
    if (impl_->timestamp_exposure_center)
    {
        const int64_t half_exposure_ns = static_cast<int64_t>(
            ctx->exposure_time.load() * 1000.0 / 2.0);
        stamp = stamp + rclcpp::Duration::from_nanoseconds(half_exposure_ns);
    }

    return stamp;
}

// ----------------------------------------------------------------------
// 单台相机的采集主循环(独立线程)
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// 软触发线程: 按目标帧率定时发软触发命令。
// 相机切到触发模式后不再自拍, 收到一个触发才出一帧 —— 帧率完全由本
// 线程决定, 与固件是否支持"帧率开关"无关, 两台相机行为严格一致。
// 重连期间句柄为空时 sendCommand 直接返回 false, 无副作用。
// ----------------------------------------------------------------------
void GalaxyCameraNode::triggerLoop(
    const std::shared_ptr<CameraContext> & ctx, size_t idx)
{
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1000000.0 / ctx->frame_rate));
    RCLCPP_INFO(
        get_logger(), "Camera '%s': trigger thread started (%.1f Hz)",
        ctx->name.c_str(), ctx->frame_rate);
    const size_t n_cam = impl_->cameras.size();
    const auto offset = period * static_cast<int64_t>(idx) / static_cast<int64_t>(n_cam > 0 ? n_cam : 1);
    auto next = std::chrono::steady_clock::now() + offset;
    std::this_thread::sleep_until(next);   // 错相: 每台错开 周期/N, 消除同发拥塞
    while (rclcpp::ok() && ctx->running)
    {
        ctx->device->sendCommand(GX_COMMAND_TRIGGER_SOFTWARE);
        next += period;
        std::this_thread::sleep_until(next);
    }
    RCLCPP_INFO(get_logger(), "Camera '%s': trigger thread exited",
                ctx->name.c_str());
}

void GalaxyCameraNode::captureLoop(const std::shared_ptr<CameraContext> & ctx)
{
    // 查询一帧的载荷大小(字节), 预分配取图缓冲。
    // 缓冲复用: 每帧直接覆盖写入, 避免频繁 new/delete。
    int64_t payload_size = 0;
    ctx->device->getInt(GX_INT_PAYLOAD_SIZE, &payload_size);
    if (payload_size <= 0)
    {
        payload_size = 1;
    }
    std::vector<uint8_t> buffer(static_cast<size_t>(payload_size));

    // 输出图像消息: 编码固定 rgb8(Bayer 已转 RGB)。
    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "rgb8";
    image_msg.header.frame_id = ctx->frame_id;

    RCLCPP_INFO(
        get_logger(), "Camera '%s': capture thread started (payload %ld bytes)",
        ctx->name.c_str(), static_cast<long>(payload_size));

    // 每轮取图/发布的具体工作放进 lambda, 由下方异常护栏统一包裹:
    // 任何一轮抛异常(OpenCV 的 cv::Exception / rclcpp 异常 / bad_alloc)
    // 都只记录详细日志并继续, 绝不让异常逃出线程函数 —— 逃逸会触发
    // std::terminate 拖死整个进程(异常安全要求)。
    auto capture_once = [&]()
    {
        // ---- 周期性健康报告: 每 30 秒一条, 让人随时知道相机活没活、好不好 ----
        const auto now_steady = std::chrono::steady_clock::now();
        if (now_steady >= ctx->next_report)
        {
            const auto elapsed = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                now_steady - (ctx->next_report - std::chrono::seconds(30)))
                .count();
            const double hz =
                elapsed > 0 ?
                ctx->stat_ok * 1000.0 / static_cast<double>(elapsed) : 0.0;
            RCLCPP_INFO(
                get_logger(),
                "Camera '%s' 健康报告: 完整帧%lu (%.1ffps) | 残帧%lu | "
                "超时%lu | 错误%lu%s",
                ctx->name.c_str(),
                static_cast<unsigned long>(ctx->stat_ok), hz,
                static_cast<unsigned long>(ctx->stat_badframe),
                static_cast<unsigned long>(ctx->stat_timeout),
                static_cast<unsigned long>(ctx->stat_error),
                ctx->stat_since_ok > 0 ? " ⚠ 当前连续无完整帧" : "");
            ctx->next_report = now_steady + std::chrono::seconds(30);
        }

        // 每次循环都重新指向当前缓冲地址(重连后缓冲可能被 resize)。
        GX_FRAME_DATA frame = {};
        frame.pImgBuf = buffer.data();
        const GX_STATUS status = ctx->device->grab(
            &frame, static_cast<uint32_t>(impl_->grab_timeout_ms));

        // ---- 成功路径: 取到一帧完整图像 ----
        if (GX_SUCCESS(status) && frame.nStatus == GX_FRAME_STATUS_SUCCESS)
        {
            ++ctx->stat_ok;
            ctx->stat_since_ok = 0;
            ctx->fail_count = 0;   // 清空失败计数
            ctx->timeout_count = 0;  // 清空超时计数

            // 按像素格式选 Bayer 排列(错配会导致颜色通道错乱)。
            // 填图像头: 时间戳 / 尺寸 / 行步长(rgb8 每像素 3 字节)。
            image_msg.header.stamp = frameTimestamp(ctx, frame.nTimestamp);
            image_msg.width = static_cast<uint32_t>(frame.nWidth);
            image_msg.height = static_cast<uint32_t>(frame.nHeight);
            image_msg.step = image_msg.width * 3;
            image_msg.data.resize(
                static_cast<size_t>(image_msg.width) * image_msg.height * 3);

            if (impl_->sim_mode)
            {
                // ---- 模拟模式分支 ----
                // SimulatedGalaxyDevice 直接输出打包 RGB24(见
                // simulated_galaxy_device.cpp 的 produceFrame), 无需
                // Bayer 解码; 拷入消息后与真机走完全相同的发布/编码路径。
                std::memcpy(
                    image_msg.data.data(), frame.pImgBuf, image_msg.data.size());
            }
            else
            {
                // ---- 真机分支: Bayer -> RGB24(邻居插值, 速度快, 适合裂缝检测) ----
                bool bayer_ok = false;
                const DX_PIXEL_COLOR_FILTER bayer =
                    bayerTypeFromPixelFormat(frame.nPixelFormat, &bayer_ok);
                if (!bayer_ok)
                {
                    // 每 5 秒最多报一次, 避免刷屏。
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "Camera '%s': unsupported pixel format %d",
                        ctx->name.c_str(), frame.nPixelFormat);
                    return;
                }
                const VxInt32 convert_status = DxRaw8toRGB24(
                    frame.pImgBuf, image_msg.data.data(),
                    static_cast<VxUint32>(frame.nWidth),
                    static_cast<VxUint32>(frame.nHeight),
                    RAW2RGB_NEIGHBOUR, bayer, false);
                if (convert_status != DX_OK)
                {
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "Camera '%s': Bayer->RGB conversion failed (%d)",
                        ctx->name.c_str(), convert_status);
                    return;
                }
            }

            // 标定信息与图像同步时间戳后发布:
            //  - raw 开启: 由 raw_publisher 连同图像一起发;
            //  - raw 关闭: 由独立 camera_info_pub 发(每帧都发, 体积极小)。
            ctx->camera_info_msg.header = image_msg.header;
            if (ctx->publish_raw)
            {
                ctx->raw_publisher.publish(image_msg, ctx->camera_info_msg);
            }
            else if (ctx->camera_info_pub)
            {
                ctx->camera_info_pub->publish(ctx->camera_info_msg);
            }

            // 再编码一版 JPEG 发到 <camera_name>/image_raw/compressed:
            // 5MP RGB 一帧 15MB 是 DDS 发布环节的主要负担(原包同样受限),
            // JPEG 只有 ~1MB, 检测链路与 rqt 订阅它都能稳定满帧。
            if (ctx->publish_compressed && ctx->compressed_pub)
            {
                const cv::Mat rgb_mat(
                    static_cast<int>(image_msg.height),
                    static_cast<int>(image_msg.width),
                    CV_8UC3, image_msg.data.data());
                const std::vector<int> encode_params = {
                    cv::IMWRITE_JPEG_QUALITY, ctx->jpeg_quality};
                if (cv::imencode(".jpg", rgb_mat, ctx->jpeg_buffer, encode_params))
                {
                    sensor_msgs::msg::CompressedImage compressed;
                    compressed.header = image_msg.header;
                    compressed.format = "jpeg";
                    compressed.data = ctx->jpeg_buffer;
                    ctx->compressed_pub->publish(compressed);
                }
                else
                {
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "Camera '%s': JPEG 编码失败", ctx->name.c_str());
                }
            }
            return;
        }

        // ---- 残帧路径: 帧到了但数据不完整 = 传输丢包(设备本身没掉线) ----
        if (GX_SUCCESS(status) && frame.nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            ++ctx->stat_badframe;
            ++ctx->stat_since_ok;
            // 残帧证明设备仍在线，只是 GVSP 数据包不完整。关闭并重开相机
            // 不会修复链路拥塞，反而会让双机同时重新发流，形成重连风暴。
            // 因此只丢弃本帧并保留诊断计数；真正掉线仍由连续超时或 SDK
            // OFFLINE/ERROR 状态触发下面的重连路径。
            ctx->fail_count = 0;
            ctx->timeout_count = 0;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Camera '%s': 收到%s (status=%d)，已丢弃但不重连。"
                "请检查 MTU/包长/网卡接收缓冲和链路带宽",
                ctx->name.c_str(), frameStatusName(frame.nStatus),
                status);
            return;
        }
        // ---- 超时路径: 偶发超时(帧率低)属正常; 连续超时 = 相机掉线, 同样要重连 ----
        else if (status == GX_STATUS_TIMEOUT)
        {
            ++ctx->stat_timeout;
            ++ctx->timeout_count;
            if (ctx->timeout_count < kMaxTimeoutCount)
            {
                return;
            }
            ctx->timeout_count = 0;
            RCLCPP_WARN(
                get_logger(),
                "Camera '%s': 连续 %d 次取图超时(无帧), 判定掉线, 准备重连..."
                "(累计超时 %lu 次; 检查网线/供电/交换机)",
                ctx->name.c_str(), kMaxTimeoutCount,
                static_cast<unsigned long>(ctx->stat_timeout));
            // 直接把失败计数顶满, 落入下面的重连流程
            ctx->fail_count = kMaxFailCount;
        }
        else if (!GX_SUCCESS(status))
        {
            ++ctx->stat_error;
        }

        // ---- 失败路径: 计数, 连续失败触发自动重连 ----
        ++ctx->fail_count;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Camera '%s': 取图失败 status=%d(%s), 连续第%d次",
            ctx->name.c_str(), status, gxStatusName(status),
            ctx->fail_count);
        if (ctx->fail_count < kMaxFailCount)
        {
            return;
        }
        ctx->fail_count = 0;

        // 重连流程(全程只影响本相机, 其它相机照常工作):
        //   停采 -> 关设备 -> 循环重开 -> 恢复参数 -> 重新开采。
        RCLCPP_WARN(
            get_logger(), "Camera '%s': trying to reconnect ...",
            ctx->name.c_str());
        ctx->device->stopAcquisition();
        ctx->device->close();

        bool reopened = false;
        while (rclcpp::ok() && ctx->running && !reopened)
        {
            std::string error;
            if (ctx->device->open(ctx->address, &error))
            {
                reopened = true;
                break;
            }
            RCLCPP_WARN(
                get_logger(),
                "Camera '%s': reconnect failed (%s), retrying in 1 s ...",
                ctx->name.c_str(), error.c_str());
            if (error.find("-5") != std::string::npos)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "  [提示] 状态码 -5 常见原因: ①寻址参数为空(直接 ros2 run 时节点名"
                    "与 yaml 命名空间不匹配, 请用 launch 或 -r __node:=galaxy_camera); "
                    "②相机正被另一个客户端独占占用。");
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (!reopened)
        {
            return;   // 收到退出信号, 直接结束线程
        }

        // 恢复相机参数并重新开始采集。
        applyCameraSettings(*ctx);
        if (!ctx->device->startAcquisition())
        {
            RCLCPP_WARN(
                get_logger(), "Camera '%s': failed to restart acquisition",
                ctx->name.c_str());
        }

        // 时间戳基准与漂移估计都作废, 下一帧重新建立
        // (避免 tick 差值与断线前混淆)。
        ctx->time_base_valid = false;
        ctx->offset_valid = false;

        // 载荷大小可能变化(理论上是常量, 防御性处理), 必要时调整缓冲。
        int64_t new_payload = 0;
        if (ctx->device->getInt(GX_INT_PAYLOAD_SIZE, &new_payload) &&
            new_payload > 0)
        {
            buffer.resize(static_cast<size_t>(new_payload));
        }
        RCLCPP_INFO(
            get_logger(), "Camera '%s': reconnected, %s",
            ctx->name.c_str(), ctx->device->describe().c_str());
    };

    while (rclcpp::ok() && ctx->running)
    {
        try
        {
            capture_once();
        }
        catch (const cv::Exception & e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Camera '%s': 采集单轮异常(cv::Exception): %s | code=%d | "
                "func=%s | file=%s:%d, 继续循环",
                ctx->name.c_str(), e.what(), e.code, e.func.c_str(),
                e.file.c_str(), e.line);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Camera '%s': 采集单轮异常(std::exception): %s, 继续循环",
                ctx->name.c_str(), e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (...)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Camera '%s': 采集单轮未知异常(非 std::exception), 继续循环",
                ctx->name.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    RCLCPP_INFO(
        get_logger(), "Camera '%s': capture thread exited",
        ctx->name.c_str());
}

// ----------------------------------------------------------------------
// 参数回调: 在线调曝光/增益; 其它参数只读(需重启生效)
// ----------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult GalaxyCameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const rclcpp::Parameter & parameter : parameters)
    {
        const std::string name = parameter.get_name();
        bool handled = false;

        // 与每台相机的参数名逐一比对。
        for (const auto & ctx : impl_->cameras)
        {
            const std::string exposure_key =
                "cameras." + ctx->name + ".exposure_time";
            const std::string gain_key = "cameras." + ctx->name + ".gain";

            // 曝光: 立即下发到相机(经 GalaxyDevice 加锁, 线程安全)。
            if (name == exposure_key &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ctx->exposure_time = parameter.as_double();
                if (!ctx->device->setFloat(
                        GX_FLOAT_EXPOSURE_TIME, ctx->exposure_time))
                {
                    result.successful = false;
                    result.reason =
                        "Failed to set exposure_time on camera " + ctx->name;
                }
                handled = true;
                break;
            }

            // 增益: 同上。
            if (name == gain_key &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                ctx->gain = parameter.as_double();
                if (!ctx->device->setFloat(GX_FLOAT_GAIN, ctx->gain))
                {
                    result.successful = false;
                    result.reason = "Failed to set gain on camera " + ctx->name;
                }
                handled = true;
                break;
            }
        }

        // 其余参数一律拒绝, 并提示需要重启。
        if (!handled)
        {
            result.successful = false;
            result.reason =
                "Parameter '" + name + "' is read-only (restart required)";
        }
    }
    return result;
}

}  // namespace galaxy_camera_dual

// ----------------------------------------------------------------------
// 注册为 rclcpp 组件: 本节点既能独立运行, 也能被 composition 容器
// 组合加载(此时库的引用计数机制保证 GXInitLib 不会冲突)。
// ----------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(galaxy_camera_dual::GalaxyCameraNode)
