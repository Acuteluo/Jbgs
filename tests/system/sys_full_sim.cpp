// 文件: tests/system/sys_full_sim.cpp
// 作用: 测试 2 —— 全模拟启动: 四模块同时存在, 三个图像话题、传感器话题、
//       状态话题均可观测(有数据且 5s 窗口内持续发布); 关停无残留(测试 8)。

#include "harness.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_interfaces/msg/env_data.hpp>
#include <std_msgs/msg/string.hpp>

using jbgs_test::Monitor;
using jbgs_test::Proc;
using jbgs_test::residue_pids;
using jbgs_test::spin_for;
using jbgs_test::wait_count;

int main()
{
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("sys_full_sim");

    // ---- 启动总 launch(无头, 全模拟) ----
    Proc launch;
    launch.start(jbgs_test::ros_prefix() +
                     "exec ros2 launch bringup all.launch.py "
                     "show_windows:=false sim:=true",
                 "sys_full_sim");

    // ---- 三个图像话题 + 传感器话题 + 状态话题均收到数据 ----
    Monitor<sensor_msgs::msg::CompressedImage> left, right, ir;
    Monitor<sensor_interfaces::msg::EnvData> env;
    Monitor<std_msgs::msg::String> core_status, insp_status;
    left.attach(node, "/left_camera/image_raw/compressed");
    right.attach(node, "/right_camera/image_raw/compressed");
    ir.attach(node, "/ir_camera/image_raw/compressed");
    env.attach(node, "/sensor/env");
    core_status.attach(node, "/core_node/status");
    insp_status.attach(node, "/inspection_node/status");

    wait_count(node, left, 5, 30.0);
    wait_count(node, right, 5, 30.0);
    wait_count(node, ir, 5, 30.0);
    wait_count(node, env, 2, 30.0);
    wait_count(node, core_status, 2, 30.0);
    wait_count(node, insp_status, 1, 30.0);

    // ---- 5 秒窗口持续发布(非一次性) ----
    const size_t l0 = left.size(), r0 = right.size(), i0 = ir.size(),
                 e0 = env.size(), c0 = core_status.size();
    spin_for(node, 5.0);
    CHECKM(left.size() - l0 >= 10, "left 5s 窗口数据不足");
    CHECKM(right.size() - r0 >= 10, "right 5s 窗口数据不足");
    CHECKM(ir.size() - i0 >= 10, "ir 5s 窗口数据不足");
    CHECKM(env.size() - e0 >= 2, "sensor 5s 窗口数据不足");
    CHECKM(core_status.size() - c0 >= 2, "core status 5s 窗口数据不足");

    // ---- 关停: SIGINT 后进程组退干净, 无残留节点(测试 8) ----
    CHECKM(launch.stop(), "launch 20s 内未响应 SIGINT");
    const auto residue = jbgs_test::wait_residue_clear();
    CHECKM(residue.empty(),
           "关停后存在残留进程: " +
               [&] {
                   std::string s;
                   for (const auto & r : residue) { s += r + "; "; }
                   return s;
               }());

    rclcpp::shutdown();
    std::cout << "sys_full_sim PASS" << std::endl;
    return 0;
}
