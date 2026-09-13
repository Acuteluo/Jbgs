// 文件: tests/system/harness.hpp
// 作用: C++ 系统测试公共设施(无 ROS 之外的第三方依赖)。
//
// 组成:
//   - CHECK/FAIL 宏: 断言失败打印文件:行并退出码 1(ctest 判定失败);
//   - Proc         : 子进程管理(bash -c 脚本, 独立会话进程组,
//                    SIGINT 整组关停, 超时 SIGKILL, 日志落盘);
//   - residue_check: 关停后残留节点进程检查(pgrep);
//   - Monitor<MsgT>: 话题订阅监听器(记录消息 + 到达时刻, 供断流分析);
//   - spin_for     : 驱动 rclcpp 处理回调指定时长;
//   - make_jpeg    : 合成 JPEG(OpenCV), 供巡检协议测试发布。
//
// 环境: ctest 通过 ENVIRONMENT 注入 JBGS_ROOT / ROS_DOMAIN_ID。

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sys/wait.h>
#include <unistd.h>

namespace jbgs_test
{

// 前置声明: 宏/Proc 在定义前会引用(头文件内的展开点早于定义点)
struct Proc;
inline std::vector<Proc *> & g_procs();
inline void jbgs_stop_all_procs();

// ------------------------------ 断言宏 ------------------------------

#define JBGS_FAIL(msg)                                              \
    do                                                              \
    {                                                               \
        std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__       \
                  << "  " << msg << std::endl;                      \
        jbgs_test::jbgs_stop_all_procs();                                      \
        std::exit(1);                                               \
    } while (0)

#define CHECK(cond)                                                 \
    do                                                              \
    {                                                               \
        if (!(cond))                                                \
        {                                                           \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__   \
                      << "  CHECK(" #cond ")" << std::endl;         \
            jbgs_test::jbgs_stop_all_procs();                                  \
            std::exit(1);                                           \
        }                                                           \
    } while (0)

#define CHECKM(cond, msg)                                           \
    do                                                              \
    {                                                               \
        if (!(cond))                                                \
        {                                                           \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__   \
                      << "  CHECK(" #cond ")  " << msg << std::endl;\
            jbgs_test::jbgs_stop_all_procs();                                  \
            std::exit(1);                                           \
        }                                                           \
    } while (0)

// ------------------------------ 子进程 ------------------------------

/// 独立会话进程组的子进程(bash -c), SIGINT 可整组关停。
struct Proc
{
    pid_t pid = -1;
    std::string log_path;

    void start(const std::string & script, const std::string & log_name)
    {
        namespace fs = std::filesystem;
        fs::create_directories("/tmp/jbgs_sys");
        log_path = "/tmp/jbgs_sys/" + log_name + ".log";
        auto * log_file = std::fopen(log_path.c_str(), "w");
        CHECKM(log_file != nullptr, std::string("打开日志失败: ") + log_path);

        g_procs().push_back(this);
        pid = fork();
        CHECKM(pid >= 0, "fork 失败");
        if (pid == 0)
        {
            setsid();                       // 独立进程组 => 可整组 SIGINT
            dup2(fileno(log_file), STDOUT_FILENO);
            dup2(fileno(log_file), STDERR_FILENO);
            fclose(log_file);
            execl("/bin/bash", "bash", "-c", script.c_str(), nullptr);
            _exit(127);                     // exec 失败
        }
        fclose(log_file);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        CHECKM(kill(-pid, 0) == 0, "子进程组未能启动: " + script);
    }

    bool alive() const
    {
        return pid > 0 && kill(-pid, 0) == 0;
    }

    /// SIGINT 整组关停; 超时未退 => SIGKILL 并返回 false(测试判失败)。
    bool stop(int timeout_ms = 20000)
    {
        if (pid <= 0)
        {
            return true;
        }
        kill(-pid, SIGINT);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (waitpid(pid, nullptr, WNOHANG) == pid)
            {
                pid = -1;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        kill(-pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        pid = -1;
        return false;
    }

    std::string log_text() const
    {
        std::ifstream f(log_path);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }
};

// 进程注册表: 断言失败退出前先 SIGINT 所有子进程, 防止残留节点污染
// 后续测试(ctest 串行, 但残留进程会跨测试存活)。
inline std::vector<Proc *> & g_procs()
{
    static std::vector<Proc *> v;
    return v;
}

inline void jbgs_stop_all_procs()
{
    for (auto * p : g_procs())
    {
        if (p != nullptr)
        {
            p->stop(8000);
        }
    }
    g_procs().clear();
}

/// source ROS2 + 工程环境的前缀(所有子进程脚本共用)。
inline std::string ros_prefix()
{
    const char * root = getenv("JBGS_ROOT");
    return std::string("set +u; source /opt/ros/humble/setup.bash && "
                       "source ") + (root ? root : "/home/cly/Jbgs") +
           "/install/setup.bash && export JBGS_ROOT=" +
           (root ? root : "/home/cly/Jbgs") + " && ";
}

/// 从 launch 日志解析 marker 行的 rclcpp 时间戳(秒, 系统钟),
/// 用作"模拟设备时间原点"(设备对象在节点内创建, 与 launch 启动时刻
/// 有 1~2s 的进程引导延迟)。
inline double log_epoch_of(const std::string & text,
                           const std::string & marker)
{
    const size_t pos = text.find(marker);
    if (pos == std::string::npos)
    {
        return -1.0;
    }
    // 时间戳在 marker 之前: "...[LEVEL] [epoch] [logger]: marker..."
    size_t close = text.rfind(']', pos);
    size_t open = (close == std::string::npos) ?
        std::string::npos : text.rfind('[', close);
    if (open == std::string::npos || close <= open + 1)
    {
        return -1.0;
    }
    // 跳过 logger 名 bracket, 找到时间戳 bracket
    if (close > 0)
    {
        const size_t close2 = text.rfind(']', open - 1);
        const size_t open2 =
            (close2 == std::string::npos) ? std::string::npos :
            text.rfind('[', close2);
        if (open2 != std::string::npos && close2 > open2 + 1)
        {
            open = open2;
            close = close2;
        }
    }
    try
    {
        return std::stod(text.substr(open + 1, close - open - 1));
    }
    catch (const std::exception &)
    {
        return -1.0;
    }
}

/// 关停后残留节点进程检查: 返回残留进程列表(空 = 干净)。
/// 注意两个坑:
///   1. pgrep -f 会匹配 popen 拉起的 sh 包装进程(cmdline 含模式串),
///      因此模式一律用括号技巧("xxx_nod[e]"), 使 sh 自身不匹配;
///   2. 僵尸进程(父进程未回收)的 /proc cmdline 为空, pgrep -f 不会
///      匹配, 但 pgrep -x 会, 故统一用 -f + 括号。
inline std::vector<std::string> residue_pids()
{
    const char * patterns[] = {
        "galaxy_camera_dual_nod[e]", "core_nod[e]", "inspection_nod[e]",
        "ir_camera_driver_nod[e]", "tas_sensor_driver_nod[e]",
        "ros2 launch bringu[p]",
    };
    std::vector<std::string> residue;
    for (const char * pat : patterns)
    {
        std::string cmd = std::string("pgrep -f '") + pat + "' 2>/dev/null";
        auto * pipe = popen(cmd.c_str(), "r");
        if (pipe == nullptr)
        {
            continue;
        }
        char buf[128];
        while (std::fgets(buf, sizeof(buf), pipe) != nullptr)
        {
            std::string line(buf);
            while (!line.empty() && (line.back() == '\n' || line.back() == ' '))
            {
                line.pop_back();
            }
            if (line.empty())
            {
                continue;
            }
            // 双重保险: 跳过僵尸(状态 Z)
            std::string stat_path = "/proc/" + line + "/stat";
            std::ifstream sf(stat_path);
            std::string word;
            std::string state;
            for (int i = 0; i < 3; ++i)
            {
                sf >> word;
                if (i == 2)
                {
                    state = word;
                }
            }
            if (state == "Z")
            {
                continue;
            }
            residue.push_back(pat + std::string(" pid=") + line);
        }
        pclose(pipe);
    }
    return residue;
}

/// SIGINT 后子节点进程可能滞后 ros2 launch 一小会儿才退出:
/// 轮询等待残留清空(超时返回最后残留, 供断言)。
inline std::vector<std::string> wait_residue_clear(double timeout = 15.0)
{
    std::vector<std::string> residue;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout));
    while (true)
    {
        residue = residue_pids();
        if (residue.empty() ||
            std::chrono::steady_clock::now() > deadline)
        {
            return residue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ------------------------------ 话题监听 ------------------------------

/// 话题监听器: 记录 (到达时刻, 消息), 供计数/断流/内容断言。
template <typename MsgT>
struct Monitor
{
    struct Item
    {
        std::chrono::steady_clock::time_point t;   // 单调钟(测时长)
        double epoch = 0.0;                        // 系统钟秒(对齐日志时间戳)
        typename MsgT::SharedPtr msg;
    };
    std::vector<Item> items;
    typename rclcpp::Subscription<MsgT>::SharedPtr sub;

    void attach(rclcpp::Node::SharedPtr node, const std::string & topic,
                bool reliable = false)
    {
        rclcpp::QoS qos(reliable ? rclcpp::QoS(30) : rclcpp::SensorDataQoS());
        sub = node->create_subscription<MsgT>(
            topic, qos, [this](typename MsgT::SharedPtr msg) {
                Item it;
                it.t = std::chrono::steady_clock::now();
                it.epoch = std::chrono::duration<double>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
                it.msg = msg;
                items.push_back(it);
            });
    }

    size_t size() const { return items.size(); }
};

/// 驱动 rclcpp 处理回调指定时长(自调用起, 而非首个消息起)。
inline void spin_for(rclcpp::Node::SharedPtr node, double seconds)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds));
    while (std::chrono::steady_clock::now() < deadline)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/// 等待监听器收到至少 n 条消息(超时失败)。
template <typename MsgT>
void wait_count(rclcpp::Node::SharedPtr node, Monitor<MsgT> & mon,
                size_t n, double timeout)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout));
    while (std::chrono::steady_clock::now() < deadline && mon.size() < n)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECKM(mon.size() >= n,
           "等待话题消息超时: 收到 " + std::to_string(mon.size()) + " 条");
}

/// 等待发布端匹配到至少 n 个订阅者(best_effort 发布必须先确认匹配,
/// 否则发布的帧会静默丢失)。
inline void wait_matched(
    rclcpp::Node::SharedPtr node,
    const std::vector<rclcpp::PublisherBase::SharedPtr> & pubs,
    double timeout)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout));
    while (std::chrono::steady_clock::now() < deadline)
    {
        rclcpp::spin_some(node);
        bool all = true;
        for (const auto & p : pubs)
        {
            if (p->get_subscription_count() == 0)
            {
                all = false;
            }
        }
        if (all)
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    JBGS_FAIL("等待发布/订阅匹配超时(检查节点是否启动、QoS 是否兼容)");
}

// ------------------------------ 其他 ------------------------------

/// 生成内容随 index 变化的合成 JPEG(尺寸小, 编码快)。
inline std::vector<uint8_t> make_jpeg(int index, int w = 64, int h = 48)
{
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(index % 256, 64, 128));
    img.at<cv::Vec3b>(0, 0) = cv::Vec3b(
        static_cast<uint8_t>((index >> 8) & 0xFF),
        static_cast<uint8_t>((index >> 16) & 0xFF),
        static_cast<uint8_t>(index & 0xFF));
    std::vector<uint8_t> buf;
    CHECK(cv::imencode(".jpg", img, buf));
    return buf;
}

}  // namespace jbgs_test
