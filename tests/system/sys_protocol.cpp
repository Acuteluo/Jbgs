// 文件: tests/system/sys_protocol.cpp
// 作用: 测试 5+6 —— 巡检协议端到端(真实 DDS + 真实写盘):
//   case A: 发布 0x01, 验证左右严格选取 t 后第 2 帧; run_save 恰有一左一右
//           文件(文件名 = 各自帧时间戳, 内容 = 该帧 JPEG 字节); 恰好一次 0x02;
//           Armed 中重复 0x01 被忽略(不产生第二次巡检)。
//   case B: 左侧第 2 帧解码失败 -> 左顺延取第 3 帧, 右仍第 2 帧(独立选帧)。
//   case C: 左侧持续离线 -> 超时放弃, 绝不发 0x02、不落盘; 恢复后下一次
//           独立巡检正常。
//   case D: 保存目录只读 -> 写盘失败不发 0x02 且半对文件被清理; 恢复
//           可写后下一巡检正常。
// 每个用例独立保存目录; 全部通过后关停节点进程并检查无残留。

#include "harness.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>

using jbgs_test::Monitor;
using jbgs_test::Proc;
using jbgs_test::residue_pids;

namespace fs = std::filesystem;

namespace
{
/// 巡检协议测试环境: 独立节点进程 + 测试进程充当左右相机/指令方。
struct Rig
{
    Proc node;
    rclcpp::Node::SharedPtr driver;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_l, pub_r;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_cmd;
    Monitor<std_msgs::msg::UInt8> ack;
    fs::path save_dir;
    int64_t stamp_ns = 0;

    void wait_ready(double timeout = 10.0)
    {
        jbgs_test::wait_matched(
            driver, {pub_l, pub_r, pub_cmd}, timeout);
    }

    void start(const std::string & name, double timeout_sec)
    {
        save_dir = fs::path("/tmp/jbgs_sys") / name;
        fs::remove_all(save_dir);
        fs::create_directories(save_dir);

        node.start(
            jbgs_test::ros_prefix() +
                "exec ros2 run culvert_inspection inspection_node --ros-args "
                "-r __node:=inspect_test -r "
                "'~/status:=/inspect_test/node_status' "
                "-p in_trulyworking:=true "
                "-p save_dir:=" + save_dir.string() + " "
                "-p frame_timeout_sec:=" + std::to_string(timeout_sec) + " "
                "-p input_topic:=/inspect_test/command "
                "-p ack_topic:=/inspect_test/ack "
                "-p left_topic:=/inspect_test/left/compressed "
                "-p right_topic:=/inspect_test/right/compressed",
            name);

        driver = rclcpp::Node::make_shared(name + "_driver");
        pub_l = driver->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/inspect_test/left/compressed", rclcpp::SensorDataQoS());
        pub_r = driver->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/inspect_test/right/compressed", rclcpp::SensorDataQoS());
        pub_cmd = driver->create_publisher<std_msgs::msg::UInt8>(
            "/inspect_test/command", rclcpp::QoS(10));
        ack.attach(driver, "/inspect_test/ack", true);
    }

    int64_t next_stamp()
    {
        stamp_ns += 100000000LL;   // 0.1s 步进, 单调唯一
        return stamp_ns;
    }

    void publish_frame(const rclcpp::Publisher<
                           sensor_msgs::msg::CompressedImage>::SharedPtr & pub,
                       int64_t stamp, const std::vector<uint8_t> & jpeg)
    {
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
        msg.header.frame_id = "test";
        msg.format = "jpeg";
        msg.data = jpeg;
        pub->publish(msg);
    }

    /// 左右同时各发一帧(时间戳相同), corrupt 侧发不可解码字节。
    void publish_pair(int64_t stamp, bool left_corrupt = false,
                      bool right_corrupt = false)
    {
        const auto jpeg = jbgs_test::make_jpeg(
            static_cast<int>(stamp % 1000000));
        publish_frame(pub_l, stamp,
                      left_corrupt ? std::vector<uint8_t>(
                          {'B', 'A', 'D', 'J', 'P', 'G'}) : jpeg);
        publish_frame(pub_r, stamp,
                      right_corrupt ? std::vector<uint8_t>(
                          {'B', 'A', 'D', 'J', 'P', 'G'}) : jpeg);
    }

    void send_trigger()
    {
        std_msgs::msg::UInt8 msg;
        msg.data = 0x01;
        pub_cmd->publish(msg);
    }

    /// 静默 + 触发 + 静默: 消除"触发前后帧"的到达边界歧义。
    void trigger_in_quiet()
    {
        jbgs_test::spin_for(driver, 0.5);
        send_trigger();
        jbgs_test::spin_for(driver, 0.5);
    }

    void spin(double sec) { jbgs_test::spin_for(driver, sec); }

    void stop()
    {
        CHECKM(node.stop(10000), "巡检节点 10s 内未退出");
        for (auto & p : jbgs_test::g_procs())
        {
            if (p == &node) { p = nullptr; }
        }
        const auto residue = jbgs_test::wait_residue_clear();
        CHECKM(residue.empty(), "关停后存在残留进程");
        driver.reset();
    }
};

/// 读取文本文件全部字节。
std::vector<uint8_t> read_file(const fs::path & p)
{
    std::ifstream f(p, std::ios::binary);
    return std::vector<uint8_t>(
        (std::istreambuf_iterator<char>(f)),
        std::istreambuf_iterator<char>());
}
}  // namespace

int main()
{
    rclcpp::init(0, nullptr);

    // ================= case A: 正常 + Armed 中重复 0x01 =================
    {
        Rig rig;
        rig.start("proto_a", 2.0);
        rig.wait_ready();   // 等节点完成 DDS 发现

        rig.trigger_in_quiet();
        rig.send_trigger();                     // Armed 中重复 0x01 -> 忽略
        rig.spin(0.1);

        int64_t s1 = 0, s2 = 0, s3 = 0;
        for (int k = 1; k <= 5; ++k)
        {
            const int64_t s = rig.next_stamp();
            if (k == 1) { s1 = s; }
            if (k == 2) { s2 = s; }
            if (k == 3) { s3 = s; }
            rig.publish_pair(s);
            rig.spin(0.15);
        }
        // 等确认(最多 5s)
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline &&
               rig.ack.size() < 1)
        {
            rclcpp::spin_some(rig.driver);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        rig.spin(0.5);   // 观察窗口: 不应再来第二个 0x02

        CHECKM(rig.ack.size() == 1,
               "0x02 次数错误: " + std::to_string(rig.ack.size()));
        CHECK(rig.ack.items[0].msg->data == 0x02);
        const auto files = [&] {
            std::vector<std::string> v;
            for (const auto & e : fs::directory_iterator(rig.save_dir))
            {
                v.push_back(e.path().filename().string());
            }
            std::sort(v.begin(), v.end());
            return v;
        }();
        CHECKM(files.size() == 2,
               "run_save 文件数错误: " + std::to_string(files.size()));
        const std::string l_name = std::to_string(s2) + "_left.jpg";
        const std::string r_name = std::to_string(s2) + "_right.jpg";
        CHECKM(files[0] == l_name, "左文件名错误: " + files[0] +
                   " (期望 " + l_name + ")");
        CHECKM(files[1] == r_name, "右文件名错误: " + files[1] +
                   " (期望 " + r_name + ")");
        // 内容 = 第 2 帧的 JPEG 字节(证明严格选帧, 非第 1/第 3 帧)
        const auto expect = jbgs_test::make_jpeg(
            static_cast<int>(s2 % 1000000));
        CHECKM(read_file(rig.save_dir / l_name) == expect, "左文件内容非第 2 帧");
        CHECKM(read_file(rig.save_dir / r_name) == expect, "右文件内容非第 2 帧");
        (void)s1; (void)s3;
        rig.stop();
        std::cout << "case A (正常选帧+重复触发忽略) PASS" << std::endl;
    }

    // ============ case B: 目标帧解码失败 -> 顺延第 3 帧 ============
    {
        Rig rig;
        rig.start("proto_b", 2.0);
        rig.wait_ready();

        rig.trigger_in_quiet();
        int64_t s2 = 0, s3 = 0;
        for (int k = 1; k <= 3; ++k)
        {
            const int64_t s = rig.next_stamp();
            if (k == 2) { s2 = s; }
            if (k == 3) { s3 = s; }
            // 左侧第 2 帧解码失败; 右侧全部有效
            rig.publish_pair(s, k == 2, false);
            rig.spin(0.15);
        }
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline &&
               rig.ack.size() < 1)
        {
            rclcpp::spin_some(rig.driver);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        CHECKM(rig.ack.size() == 1, "case B 0x02 次数错误");
        // 左 = 第 3 帧(顺延), 右 = 第 2 帧(独立选帧)
        CHECKM(fs::exists(rig.save_dir / (std::to_string(s3) + "_left.jpg")),
               "左侧未顺延取第 3 帧");
        CHECKM(fs::exists(rig.save_dir / (std::to_string(s2) + "_right.jpg")),
               "右侧未取第 2 帧");
        size_t n_files = 0;
        for (const auto & e : fs::directory_iterator(rig.save_dir)) { ++n_files; }
        CHECKM(n_files == 2, "case B 文件数错误");
        rig.stop();
        std::cout << "case B (解码失败顺延) PASS" << std::endl;
    }

    // ============ case C: 左侧持续离线 -> 超时放弃 -> 恢复 ============
    {
        Rig rig;
        rig.start("proto_c", 1.2);
        rig.wait_ready();

        rig.trigger_in_quiet();
        for (int k = 1; k <= 4; ++k)     // 只有右侧出图
        {
            int64_t s = rig.next_stamp();
            const auto jpeg = jbgs_test::make_jpeg(
                static_cast<int>(s % 1000000));
            rig.publish_frame(rig.pub_r, s, jpeg);
            rig.spin(0.2);
        }
        rig.spin(2.5);                   // 越过 1.2s 超时窗
        CHECKM(rig.ack.size() == 0, "左侧离线超时却发了 0x02");
        size_t n_files = 0;
        for (const auto & e : fs::directory_iterator(rig.save_dir)) { ++n_files; }
        CHECKM(n_files == 0, "超时周期却落了盘");

        // 恢复后的下一次独立巡检正常
        rig.trigger_in_quiet();
        int64_t s2 = 0;
        for (int k = 1; k <= 3; ++k)
        {
            const int64_t s = rig.next_stamp();
            if (k == 2) { s2 = s; }
            rig.publish_pair(s);
            rig.spin(0.15);
        }
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline &&
               rig.ack.size() < 1)
        {
            rclcpp::spin_some(rig.driver);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        CHECKM(rig.ack.size() == 1, "恢复后巡检未发 0x02");
        CHECKM(fs::exists(rig.save_dir / (std::to_string(s2) + "_left.jpg")) &&
                   fs::exists(rig.save_dir /
                              (std::to_string(s2) + "_right.jpg")),
               "恢复后巡检文件不齐");
        rig.stop();
        std::cout << "case C (左侧离线超时+恢复) PASS" << std::endl;
    }

    // ============ case D: 写盘失败(只读目录) -> 不发 0x02 -> 恢复 ============
    {
        Rig rig;
        rig.start("proto_d", 2.0);
        rig.wait_ready();

        // 目录改只读: 写盘必失败(目录已存在, create_directories 不报错)
        permissions(rig.save_dir, fs::perms::owner_read |
                    fs::perms::owner_exec | fs::perms::group_read |
                    fs::perms::group_exec | fs::perms::others_read |
                    fs::perms::others_exec);

        rig.trigger_in_quiet();
        int64_t s2 = 0;
        for (int k = 1; k <= 3; ++k)
        {
            const int64_t s = rig.next_stamp();
            if (k == 2) { s2 = s; }
            rig.publish_pair(s);
            rig.spin(0.15);
        }
        rig.spin(1.0);
        CHECKM(rig.ack.size() == 0, "写盘失败却发了 0x02");
        size_t n_files = 0;
        for (const auto & e : fs::directory_iterator(rig.save_dir)) { ++n_files; }
        CHECKM(n_files == 0, "写盘失败却残留半对文件");

        // 恢复可写 -> 下一巡检正常
        permissions(rig.save_dir, fs::perms::owner_all);
        rig.trigger_in_quiet();
        int64_t s2b = 0;
        for (int k = 1; k <= 3; ++k)
        {
            const int64_t s = rig.next_stamp();
            if (k == 2) { s2b = s; }
            rig.publish_pair(s);
            rig.spin(0.15);
        }
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline &&
               rig.ack.size() < 1)
        {
            rclcpp::spin_some(rig.driver);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        CHECKM(rig.ack.size() == 1, "恢复后巡检未发 0x02");
        CHECKM(fs::exists(rig.save_dir / (std::to_string(s2b) + "_left.jpg")),
               "恢复后左文件缺失");
        rig.stop();
        std::cout << "case D (写盘失败+恢复) PASS" << std::endl;
    }

    rclcpp::shutdown();
    std::cout << "sys_protocol PASS" << std::endl;
    return 0;
}
