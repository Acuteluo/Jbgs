// Copyright (c) 2024, galaxy_camera_dual contributors
//
// 文件: galaxy_camera_node.hpp
// 作用: 多相机驱动节点 GalaxyCameraNode 的声明。
//
// 节点职责:
//  - 一个节点同时启动并管理一台或多台大恒相机(默认左/右两台);
//  - 每台相机对应一组参数(cameras.<name>.*), 可分别用
//    序列号/UserID/IP/设备序号 寻址;
//  - 每台相机有自己的采集线程、话题、frame_id 与标定文件。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace galaxy_camera_dual
{

// 前置声明: 完整定义在 galaxy_camera_node.cpp 里。
// 头文件里只使用 指针/引用, 不暴露内部实现细节(Pimpl 手法的一部分)。
struct CameraContext;

/// 大恒 Galaxy 多相机驱动节点。
///
/// 线程模型(与单相机原包的关键区别):
///  - 每台相机由独立的 std::thread 负责阻塞式取图(GXGetImage),
///    一台相机的等待绝不会拖慢另一台;
///  - 节点由 MultiThreadedExecutor 驱动(见 CMakeLists.txt 的
///    EXECUTOR MultiThreadedExecutor), 参数回调/服务在执行器线程跑,
///    与取图线程完全分开;
///  - 所有 SDK 访问都经过 GalaxyDevice 封装: 每个设备一把互斥锁,
///    取图线程与执行器线程不会在同一 GX_DEV_HANDLE 上竞争;
///  - 某台相机掉线时, 由其自己的采集线程自动重连, 另一台不受影响;
///  - 析构时按固定顺序收尾: 置停止标志 -> 停采(唤醒阻塞取图)
///    -> join 线程 -> 关设备 -> 关库, 全程无死锁、无残留线程。
class GalaxyCameraNode : public rclcpp::Node
{
public:
    /// 构造函数: 完成 库初始化 -> 声明参数 -> 枚举设备 -> 打开全部相机
    /// -> 注册参数回调 -> 启动每台相机的采集线程 这一整套流程。
    /// 中途若收到 Ctrl+C(rclcpp::ok() 为 false)会优雅提前返回。
    explicit GalaxyCameraNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~GalaxyCameraNode() override;

private:
    /// 把参数里配置的曝光/增益/帧率/UserID 等一次性下发给相机。
    /// 构造函数打开相机后、以及断线重连成功后都会调用。
    void applyCameraSettings(CameraContext & ctx);

    /// 把相机硬件 tick 计数值换算成 ROS 时间戳。
    /// nTimestamp 是相机在"曝光开始(帧开始)"瞬间锁存的 tick 值。
    /// 帧间间隔由相机晶振决定(微秒级抖动), 比"ROS 收到图像的时刻"
    /// 干净得多; 可选做两处修正:
    ///  - 时钟漂移补偿(默认开): 用 EMA 平滑"到达时刻-相机侧时刻"的
    ///    偏差, 让时间戳长期跟随主机时钟, 与里程计/IMU 对齐;
    ///  - 曝光中心修正(默认关): +曝光时间/2, 运动补偿场景更标准。
    rclcpp::Time frameTimestamp(
        const std::shared_ptr<CameraContext> & ctx, uint64_t device_ticks);

    /// 单台相机的采集主循环(运行在独立的 std::thread 里):
    /// 取图 -> Bayer 转 RGB -> 打时间戳 -> 发布; 连续失败触发自动重连。
    void captureLoop(const std::shared_ptr<CameraContext> & ctx);
    void triggerLoop(const std::shared_ptr<CameraContext> & ctx, size_t idx);
    void hotPlugLoop();                      ///< 热插拔守望线程(缺席相机补开)
    std::atomic<size_t> trigger_index_{0};   ///< 触发线程序号(错相用)

    /// 参数回调: 支持运行时在线修改 cameras.<name>.exposure_time 与
    /// cameras.<name>.gain, 其它参数视为只读(需重启)。
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);

    struct Impl;                 ///< 实现细节全部藏在 .cpp(Pimpl 手法)
    std::unique_ptr<Impl> impl_;
};

}  // namespace galaxy_camera_dual
