// 文件: tests/system/sys_hotplug.cpp
// 作用: 测试 4 —— 分别模拟四类设备运行中断连和恢复(一次启动按时间错开):
//         8~12s 左大恒, 14~18s 右大恒, 20~24s 红外, 26~30s 传感器。
//       检查: 断连窗口内该设备零数据、其余设备持续发布; 窗口后自动恢复;
//       core_node 状态行在窗口内标该路 OFF; 日志含断连告警与恢复日志
//       (节流, 不刷屏); 关停无残留。

#include "harness.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_interfaces/msg/env_data.hpp>
#include <std_msgs/msg/string.hpp>

using jbgs_test::Monitor;
using jbgs_test::Proc;
using jbgs_test::residue_pids;

namespace
{
struct Window
{
    std::string name;
    double t0;    // 相对启动时刻(秒)
    double t1;
};
const std::vector<Window> kWindows = {
    {"left", 8.0, 12.0}, {"right", 14.0, 18.0},
    {"ir", 20.0, 24.0}, {"sensor", 26.0, 30.0},
};

/// 设备名 -> 监听器指针(通用访问)。
template <typename T>
size_t count_in_window(const std::vector<std::chrono::steady_clock::time_point> & times,
                       const T & base, double t0, double t1)
{
    size_t n = 0;
    for (const auto & t : times)
    {
        const double dt = std::chrono::duration<double>(t - base).count();
        if (dt >= t0 && dt < t1)
        {
            ++n;
        }
    }
    return n;
}
}  // namespace

int main()
{
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("sys_hotplug");

    Proc launch;
    launch.start(jbgs_test::ros_prefix() +
                     "exec ros2 launch bringup all.launch.py "
                     "show_windows:=false sim:=true absent:=2 "
                     "left_disc_at:=8 left_disc_dur:=4 "
                     "right_disc_at:=14 right_disc_dur:=4 "
                     "ir_disc_at:=20 ir_disc_dur:=4 "
                     "sensor_disc_at:=26 sensor_disc_dur:=4",
                 "sys_hotplug");

    // ---- 全部话题全程监听(带到达时刻) ----
    Monitor<sensor_msgs::msg::CompressedImage> left, right, ir;
    Monitor<sensor_interfaces::msg::EnvData> env;
    Monitor<std_msgs::msg::String> status;
    left.attach(node, "/left_camera/image_raw/compressed");
    right.attach(node, "/right_camera/image_raw/compressed");
    ir.attach(node, "/ir_camera/image_raw/compressed");
    env.attach(node, "/sensor/env");
    status.attach(node, "/core_node/status");

    // ---- 时间原点: 从 launch 日志解析 galaxy 模拟设备创建时刻 ----
    // (模拟设备的时间窗以设备对象构造为 0 点, 与 launch 启动时刻有
    //  1~2s 进程引导差; 用日志时间戳对齐才能精确断言断连窗口)
    double origin = -1.0;
    {
        const auto ready = std::chrono::steady_clock::now() +
            std::chrono::seconds(30);
        while (std::chrono::steady_clock::now() < ready)
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            origin = jbgs_test::log_epoch_of(
                launch.log_text(), "★ 模拟模式");
            if (origin > 0)
            {
                break;
            }
        }
        CHECKM(origin > 0, "launch 日志未找到 galaxy 模拟模式起始时间戳");
    }

    // 各设备相对原点的消息时刻(ir/sensor 节点引导差 <0.5s, 窗口各留 1s 余量)
    const auto epochs_of = [&](const std::string & dev) {
        std::vector<double> v;
        if (dev == "left")
        {
            for (const auto & it : left.items) { v.push_back(it.epoch - origin); }
        }
        else if (dev == "right")
        {
            for (const auto & it : right.items) { v.push_back(it.epoch - origin); }
        }
        else if (dev == "ir")
        {
            for (const auto & it : ir.items) { v.push_back(it.epoch - origin); }
        }
        else
        {
            for (const auto & it : env.items) { v.push_back(it.epoch - origin); }
        }
        return v;
    };

    // 等到 37s(覆盖 4 个断连窗 + 各自恢复), 以原点为基准
    while (true)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const double now_rel = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count() -
            origin;
        if (now_rel >= 37.0)
        {
            break;
        }
    }

    // ---- 每个断连窗口: 本设备零数据; 其余设备持续发布 ----
    for (const auto & w : kWindows)
    {
        const auto self_t = epochs_of(w.name);
        size_t n_self = 0;
        for (double t : self_t)
        {
            if (t >= w.t0 + 1.0 && t < w.t1 - 1.0)
            {
                ++n_self;
            }
        }
        CHECKM(n_self == 0,
               w.name + " 断连窗口内仍收到 " +
                   std::to_string(n_self) + " 条数据");
        for (const auto & other : kWindows)
        {
            if (other.name == w.name)
            {
                continue;
            }
            const auto other_t = epochs_of(other.name);
            size_t n_other = 0;
            for (double t : other_t)
            {
                if (t >= w.t0 + 1.0 && t < w.t0 + 4.0)
                {
                    ++n_other;
                }
            }
            const size_t minimum = other.name == "sensor" ? 2 : 20;
            CHECKM(n_other >= minimum,
                   w.name + " 断连期间 " + other.name + " 仅 " +
                       std::to_string(n_other) + " 条, 其他设备未保持独立");
        }
    }

    // ---- 恢复: 各设备在其断连窗结束后重新出数据 ----
    for (const auto & w : kWindows)
    {
        const auto t = epochs_of(w.name);
        size_t n = 0;
        for (double x : t)
        {
            if (x >= w.t1 + 1.0 && x < w.t1 + 6.0)
            {
                ++n;
            }
        }
        const size_t minimum = w.name == "sensor" ? 2 : 20;
        CHECKM(n >= minimum,
               w.name + " 断连恢复后 5s 仅 " + std::to_string(n) + " 条");
    }

    // ---- core_node 状态行: 断连窗口内应出现该路 OFF 标注 ----
    // (标注出现时刻 = 断连后 stale_timeout 3s, 窗口上沿放宽 3.5s)
    bool left_off = false;
    bool sensor_off = false;
    for (const auto & it : status.items)
    {
        const double dt = it.epoch - origin;
        if (dt >= kWindows[0].t0 && dt <= kWindows[0].t1 + 3.5 &&
            it.msg->data.find("left_hz=0(OFF)") != std::string::npos)
        {
            left_off = true;
        }
        if (dt >= kWindows[3].t0 && dt <= kWindows[3].t1 + 3.5 &&
            it.msg->data.find("env[no_data]") != std::string::npos)
        {
            sensor_off = true;
        }
    }
    CHECKM(left_off, "left 断连窗口内 core status 未标 OFF");
    CHECKM(sensor_off, "sensor 断连窗口内 core status 未标 no_data");
    // ---- 日志: 断连告警 + 恢复日志(节流) ----
    const std::string log = launch.log_text();
    CHECKM(log.find("判定掉线") != std::string::npos ||
               log.find("取图失败") != std::string::npos,
           "日志缺少大恒断连告警");
    CHECKM(log.find("红外相机连续") != std::string::npos &&
               log.find("判定离线") != std::string::npos,
           "日志缺少红外断连告警");
    CHECKM(log.find("红外相机已接入") != std::string::npos,
           "日志缺少红外恢复日志");
    CHECKM(log.find("串口已恢复") != std::string::npos,
           "日志缺少传感器恢复日志");

    CHECKM(launch.stop(), "launch 未正常关停");
    const auto residue = jbgs_test::wait_residue_clear();
    CHECKM(residue.empty(), "关停后存在残留进程");
    rclcpp::shutdown();
    std::cout << "sys_hotplug PASS" << std::endl;
    return 0;
}
