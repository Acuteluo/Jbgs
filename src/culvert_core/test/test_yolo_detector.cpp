// Copyright (c) 2024, culvert_core contributors
//
// 文件: test_yolo_detector.cpp
// 作用: YoloDetector 加载 + 推理回归(OpenCV 4.5.4 必须能吃当前 ONNX)。
//
// 不弹窗、不依赖真机。模型与实拍图都在仓库 models/、pictures/ 内。

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "culvert_core/yolo_detector.hpp"

namespace
{

std::string RepoFile(const std::string & rel)
{
    const char * root = std::getenv("JBGS_ROOT");
    if (root == nullptr || root[0] == '\0')
    {
        return rel;
    }
    return std::string(root) + "/" + rel;
}

bool FileExists(const std::string & path)
{
    std::ifstream f(path.c_str());
    return f.good();
}

void EnsureRclcpp()
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
}

}  // namespace

TEST(YoloDetector, SegModelLoadsAndDetectsRealCrack)
{
    EnsureRclcpp();
    const std::string model = RepoFile("models/crack_seg_best.onnx");
    const std::string image = RepoFile(
        "pictures/light/capture_20260823_172710_left_632.jpg");
    ASSERT_TRUE(FileExists(model)) << "缺少模型: " << model;
    ASSERT_TRUE(FileExists(image)) << "缺少实拍图: " << image;

    culvert_core::YoloDetector detector(
        model, false, culvert_core::YoloDetector::ModelType::Seg);
    ASSERT_TRUE(detector.IsReady())
        << "OpenCV DNN 未能加载分割模型(检查是否经过 scripts/make_opencv_onnx.py)";

    const cv::Mat img = cv::imread(image, cv::IMREAD_COLOR);
    ASSERT_FALSE(img.empty()) << "读图失败: " << image;

    const std::vector<culvert_core::CrackObject> results = detector.Detect(img);
    ASSERT_FALSE(results.empty()) << "已知裂缝图应至少检出 1 个框";
    EXPECT_GE(results.front().confidence, 0.35f);
    EXPECT_FALSE(results.front().mask.empty()) << "seg 模型应产出掩膜";
    EXPECT_EQ(results.front().mask.size(), img.size());
}

TEST(YoloDetector, DetectBlankImageDoesNotCrash)
{
    EnsureRclcpp();
    const std::string model = RepoFile("models/crack_seg_best.onnx");
    ASSERT_TRUE(FileExists(model)) << "缺少模型: " << model;

    culvert_core::YoloDetector detector(
        model, false, culvert_core::YoloDetector::ModelType::Seg);
    ASSERT_TRUE(detector.IsReady());

    const cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(114, 114, 114));
    const std::vector<culvert_core::CrackObject> results = detector.Detect(blank);
    EXPECT_LE(static_cast<int>(results.size()), 50);
}
