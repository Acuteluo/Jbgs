// Copyright (c) 2024, culvert_core contributors
//
// 文件: test_yolo.cpp
// 作用: 单张图片 YOLO 裂缝检测测试工具(imshow 显示检测框)。
//
// 用法:
//   ros2 run culvert_core test_yolo <图片路径> [模型路径] [conf阈值] [模型类型]
//
// 示例:
//   # 默认(seg 分割模型, 框 + 掩膜描边):
//   ros2 run culvert_core test_yolo picture.jpg
//   # 显式指定 detect 检测模型(仅框):
//   ros2 run culvert_core test_yolo picture.jpg \
//       models/crack_best.onnx 0.35 detect
//
// 功能:
//   1. 读入一张图片(任意尺寸, 内部自动做 letterbox 预处理);
//   2. 加载 YOLO 模型检测裂缝(detect 或 seg);
//   3. 在原图上画检测框与置信度(seg 额外描边), 用 imshow 弹窗显示;
//   4. 按任意键关闭窗口, 同时把结果存到图片同目录的
//      <名>_yolo_result.jpg 便于复查。
//
// 说明:
//   本工具独立于 core_node 运行, 方便单图排查检测/画框是否正确。
//   检测核心复用 yolo_detector + yolo_crack, 与正式节点完全一致。

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "culvert_core/yolo_crack.hpp"
#include "culvert_core/yolo_detector.hpp"

int main(int argc, char ** argv)
{
    // 仅初始化 rclcpp 日志系统(本工具不创建节点)
    rclcpp::init(argc, argv);

    // ---- 1. 解析命令行参数 ----
    const std::string usage =
        "用法: test_yolo <图片路径> [模型路径] [conf阈值] [模型类型]\n"
        "  模型类型: seg(框+掩膜描边, 默认) / detect(仅框)\n"
        "  默认: 模型 = Jbgs/models/crack_seg_best.onnx, conf = 0.35\n"
        "  例 : test_yolo picture.jpg\n"
        "       test_yolo picture.jpg "
        "models/crack_best.onnx 0.35 detect\n";
    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("test_yolo"), "%s", usage.c_str());
        rclcpp::shutdown();
        return 1;
    }

    const std::string img_path = argv[1];
    // 默认用 seg 分割模型(与 config/core_node_params.yaml 的默认一致);
    // 想用旧 detect 模型时, 显式传 crack_best.onnx 并把模型类型写成 detect。
    const std::string model_path =
        (argc >= 3) ? argv[2] : "models/crack_seg_best.onnx";
    const float conf_threshold = (argc >= 4) ? std::atof(argv[3]) : 0.35f;
    const std::string model_type_str = (argc >= 5) ? argv[4] : "seg";
    const culvert_core::YoloDetector::ModelType model_type =
        (model_type_str == "seg") ?
        culvert_core::YoloDetector::ModelType::Seg :
        culvert_core::YoloDetector::ModelType::Detect;

    // ---- 2. 读图 ----
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (img.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("test_yolo"),
            "无法读取图片: %s", img_path.c_str());
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("test_yolo"),
        "读入图片 %s, 尺寸 = %d x %d", img_path.c_str(),
        img.cols, img.rows);

    // ---- 3. 初始化检测器(CPU 模式) ----
    culvert_core::YoloDetector detector(model_path, false, model_type);
    if (!detector.IsReady())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("test_yolo"), "模型加载失败: %s",
            model_path.c_str());
        rclcpp::shutdown();
        return 1;
    }
    detector.score_threshold_ = conf_threshold;

    // ---- 4. 推理计时并检测 ----
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<culvert_core::CrackObject> results = detector.Detect(img);
    const auto t1 = std::chrono::steady_clock::now();
    const double infer_ms = std::chrono::duration<double, std::milli>(
        t1 - t0).count();

    RCLCPP_INFO(
        rclcpp::get_logger("test_yolo"),
        "检出 %zu 个裂缝(%s), 推理耗时 %.1f ms",
        results.size(), model_type_str.c_str(), infer_ms);
    // 打印每个框坐标与掩膜情况, 便于核对画框是否正确
    for (const auto & obj : results)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("test_yolo"),
            "  框: x=%d y=%d w=%d h=%d conf=%.3f mask=%s",
            obj.box.x, obj.box.y, obj.box.width, obj.box.height,
            obj.confidence,
            obj.mask.empty() ? "无" : "有");
    }

    // ---- 5. 画框 + imshow 显示 ----
    cv::Mat show = img.clone();
    culvert_core::DrawCrackResults(show, results, infer_ms, 0.0);

    cv::namedWindow("test_yolo", cv::WINDOW_NORMAL);
    cv::imshow("test_yolo", show);
    RCLCPP_INFO(
        rclcpp::get_logger("test_yolo"),
        "已弹出显示窗口, 按任意键关闭 (结果同时保存到 test_yolo_result.jpg)");

    // 等待按键; 返回的是按下的 ASCII 码(255 表示窗口被关闭)
    const int key = cv::waitKey(0);
    cv::destroyAllWindows();
    if (key == 27)   // ESC
    {
        RCLCPP_INFO(rclcpp::get_logger("test_yolo"), "检测到 ESC, 退出");
    }

    // ---- 6. 结果存到图片同目录, 便于复查 ----
    const std::string out_path =
        img_path.substr(0, img_path.find_last_of('.')) + "_yolo_result.jpg";
    cv::imwrite(out_path, show);
    RCLCPP_INFO(
        rclcpp::get_logger("test_yolo"),
        "检测结果已保存到: %s", out_path.c_str());

    rclcpp::shutdown();
    return 0;
}
