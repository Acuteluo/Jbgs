// 文件: tests/system/sys_off_mode.cpp
// 作用: 测试 7 —— in_trulyworking=false: 巡检协议完全不工作。
//       发布 0x01 后: 不产生保存文件、不发布 0x02; 节点日志明确提示
//       未订阅协议话题; 其余感知模块照常工作。

#include "harness.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <filesystem>

using jbgs_test::Monitor;
using jbgs_test::Proc;
using jbgs_test::residue_pids;

namespace fs = std::filesystem;

int main()
{
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("sys_off_mode");

    // 巡检保存目录临时指向独立目录, 便于断言"无新文件"
    const fs::path save_dir = "/tmp/jbgs_sys/off_mode_save";
    fs::remove_all(save_dir);
    fs::create_directories(save_dir);

    Proc launch;
    launch.start(jbgs_test::ros_prefix() +
                     "exec ros2 launch bringup all.launch.py "
                     "show_windows:=false sim:=true in_trulyworking:=false "
                     "save_dir:=" + save_dir.string(),
                 "sys_off_mode");

    // 其余模块正常工作
    Monitor<sensor_msgs::msg::CompressedImage> left;
    left.attach(node, "/left_camera/image_raw/compressed");
    const auto flow_deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < flow_deadline && left.size() < 5)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECKM(left.size() >= 5, "感知模块未正常工作");

    // 订阅确认话题(不应有任何消息)
    Monitor<std_msgs::msg::UInt8> ack;
    ack.attach(node, "/inspection/ack", true);

    // 发布 0x01(协议已关闭, 无订阅者)
    auto cmd_pub = node->create_publisher<std_msgs::msg::UInt8>(
        "/inspection/command", rclcpp::QoS(10));
    // 稍等以确保若节点误订阅也能建立匹配
    jbgs_test::spin_for(node, 1.0);
    std_msgs::msg::UInt8 msg;
    msg.data = 0x01;
    cmd_pub->publish(msg);
    jbgs_test::spin_for(node, 3.0);

    CHECKM(ack.size() == 0, "in_trulyworking=false 却收到了 0x02");
    size_t n_files = 0;
    for (const auto & e : fs::directory_iterator(save_dir)) { ++n_files; }
    CHECKM(n_files == 0, "in_trulyworking=false 却保存了巡检图片");

    // 节点日志应有明确提示
    const std::string log = launch.log_text();
    CHECKM(log.find("in_trulyworking=false") != std::string::npos,
           "日志缺少巡检关闭提示");

    CHECKM(launch.stop(), "launch 未正常关停");
    const auto residue = jbgs_test::wait_residue_clear();
    CHECKM(residue.empty(), "关停后存在残留进程");
    rclcpp::shutdown();
    std::cout << "sys_off_mode PASS" << std::endl;
    return 0;
}
