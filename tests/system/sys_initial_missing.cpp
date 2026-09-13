// 文件: tests/system/sys_initial_missing.cpp
// 作用: 测试 3 —— 初始缺少左/右/红外/传感器四种情形分别验证:
//       其余模块持续工作, 缺席设备无数据, core_node 状态标 OFF。
// 用法: sys_initial_missing <left|right|ir|sensor>   (ctest 传参)

#include "harness.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_interfaces/msg/env_data.hpp>
#include <std_msgs/msg/string.hpp>

using jbgs_test::Monitor;
using jbgs_test::Proc;
using jbgs_test::residue_pids;
using jbgs_test::spin_for;

int main(int argc, char ** argv)
{
    CHECKM(argc == 2, "用法: sys_initial_missing <left|right|ir|sensor>");
    const std::string missing = argv[1];
    CHECKM(missing == "left" || missing == "right" || missing == "ir" ||
               missing == "sensor",
           "未知设备名: " + missing);

    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("sys_initial_missing");

    // 缺席设备整个测试期间保持"未接入"(120s)
    Proc launch;
    launch.start(jbgs_test::ros_prefix() +
                     "exec ros2 launch bringup all.launch.py "
                     "show_windows:=false sim:=true " +
                     missing + "_absent:=120",
                 "sys_initial_missing_" + missing);

    Monitor<sensor_msgs::msg::CompressedImage> left, right, ir;
    Monitor<sensor_interfaces::msg::EnvData> env;
    Monitor<std_msgs::msg::String> status;
    left.attach(node, "/left_camera/image_raw/compressed");
    right.attach(node, "/right_camera/image_raw/compressed");
    ir.attach(node, "/ir_camera/image_raw/compressed");
    env.attach(node, "/sensor/env");
    status.attach(node, "/core_node/status");

    // ---- 其余模块先出首帧(上限 30s, 兼顾 2s 初始缺失) ----
    const auto wait_first = [&](Monitor<sensor_msgs::msg::CompressedImage> & m,
                                const char * name) {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(30);
        while (std::chrono::steady_clock::now() < deadline && m.size() < 3)
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        CHECKM(m.size() >= 3,
               std::string("其余设备 ") + name + " 未出数据(被缺席设备拖累?)");
    };
    if (missing != "left") { wait_first(left, "left"); }
    if (missing != "right") { wait_first(right, "right"); }
    if (missing != "ir") { wait_first(ir, "ir"); }

    // ---- 缺席设备 5s 零数据; 其余设备持续发布 ----
    const size_t l0 = left.size(), r0 = right.size(), i0 = ir.size(),
                 e0 = env.size();
    spin_for(node, 5.0);
    if (missing == "sensor")
    {
        CHECKM(env.size() - e0 == 0, "传感器缺席却收到数据");
    }
    else
    {
        Monitor<sensor_msgs::msg::CompressedImage> * m =
            missing == "left" ? &left :
            missing == "right" ? &right : &ir;
        CHECKM(m->size() - (missing == "left" ? l0 :
                            missing == "right" ? r0 : i0) == 0,
               missing + " 声称缺席却收到数据");
    }
    if (missing != "left")
    {
        CHECKM(left.size() - l0 >= 10, "left 持续性不足");
    }
    if (missing != "right")
    {
        CHECKM(right.size() - r0 >= 10, "right 持续性不足");
    }
    if (missing != "ir")
    {
        CHECKM(ir.size() - i0 >= 10, "ir 持续性不足");
    }
    if (missing != "sensor")
    {
        CHECKM(env.size() - e0 >= 2, "sensor 持续性不足");
    }

    // ---- core_node 状态: 缺席路标 OFF / env 无数据(15s 内必有 1Hz 状态) ----
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() < deadline)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    bool marked = false;
    for (const auto & it : status.items)
    {
        const std::string & s = it.msg->data;
        if (missing == "sensor" &&
            s.find("env[no_data]") != std::string::npos)
        {
            marked = true;
        }
        if (missing == "left" &&
            s.find("left_hz=0(OFF)") != std::string::npos)
        {
            marked = true;
        }
        if (missing == "right" &&
            s.find("right_hz=0(OFF)") != std::string::npos)
        {
            marked = true;
        }
        if (missing == "ir" && s.find("ir_hz=0(OFF)") != std::string::npos)
        {
            marked = true;
        }
    }
    CHECKM(marked, "core_node 状态未标注缺席设备 OFF: " +
                       (status.size() ?
                            status.items.back().msg->data : "<无状态>"));

    CHECKM(launch.stop(), "launch 未正常关停");
    const auto residue = jbgs_test::wait_residue_clear();
    CHECKM(residue.empty(), "关停后存在残留进程");
    rclcpp::shutdown();
    std::cout << "sys_initial_missing(" << missing << ") PASS" << std::endl;
    return 0;
}
