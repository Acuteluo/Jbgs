// Copyright (c) 2024, culvert_core contributors
//
// 文件: yolo_detector.hpp
// 作用: 涵洞裂缝 YOLO 模型推理封装(基于 OpenCV DNN)。
//
// 仿照旧项目 /home/k/project/src/img_processing 里的 yolo_detector,
// 但针对"单类裂缝检测"做了适配。支持两种模型(通过 model_type 切换):
//   - 检测模型(detect, 默认, 对应 crack_best.onnx):
//       输出 [1, 5, 8400], 只有矩形框;
//       第 4 列是裂缝置信度(导出时已 sigmoid, 直接用);
//       前 4 列是归一化中心点 cx/cy 与宽高 w/h(缩放回原图);
//   - 分割模型(seg, 对应 crack_seg_best.onnx):
//       输出 [1, 37, 8400] + [1, 32, 160, 160], 除矩形框外还有裂缝掩膜;
//       前 4 列是框坐标, 第 4 列置信度, 第 5~36 列是 32 个掩膜系数,
//       第二个输出是 32 张 160x160 的原型掩膜(proto mask);
//       掩膜 = 系数 与 原型掩膜 线性组合 -> >0 二值化 -> 缩放到原图。
//
// 两种模型都用 letterbox 预处理(等比缩放+灰边), 与训练完全一致。
// 支持 CPU / GPU(OpenVINO) 双模式, 若 GPU 不可用自动回退 CPU。
//
// OpenCV 4.5.4: 仓库 models/crack_*_best.onnx 已按
// scripts/make_opencv_onnx.py 做过等价图变换(折叠动态 chunk、
// 把 stride 乘改到 channel 轴), 输出仍是 [1,5|37,8400](+/proto)。

#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace culvert_core
{

/// 单个裂缝检测结果(由 YoloDetector 解析模型输出后填充)。
struct CrackObject
{
    cv::Rect box;      ///< 在原图坐标系下的矩形框
    float confidence;  ///< 裂缝置信度(0~1, 已 sigmoid)
    int class_id;      ///< 类别 id(本数据集只有 0 = Crack)
    cv::Mat mask;      ///< 分割掩膜(原图大小, CV_8UC1, 0/255);
                       ///< 检测模型(detect)无掩膜, 此为空 Mat
};

/**
 * @brief 裂缝 YOLO 检测器(OpenCV DNN 封装)
 *
 * 负责: 加载 ONNX 模型 -> 图像预处理(letterbox blob) -> 前向推理 ->
 *       后处理(缩放坐标 + 置信度过滤 + NMS, 分割模型额外解码掩膜)
 *       -> 返回 CrackObject 列表。
 */
class YoloDetector
{
public:
    /// 模型类型: detect = 只有框; seg = 框 + 分割掩膜
    enum class ModelType
    {
        Detect,
        Seg,
    };

    /**
     * @brief 构造, 加载模型
     * @param model_path ONNX 模型绝对路径
     * @param use_gpu    是否优先使用 GPU(OpenVINO); false=CPU(默认)
     * @param model_type 模型类型: Detect(默认) / Seg
     */
    YoloDetector(
        const std::string & model_path, bool use_gpu,
        ModelType model_type = ModelType::Detect);

    /**
     * @brief 执行一帧图像的裂缝检测
     * @param img BGR 原图(任意分辨率)
     * @return std::vector<CrackObject> 检测到的裂缝集合(按置信度降序)
     */
    std::vector<CrackObject> Detect(const cv::Mat & img);

    /// 模型是否加载成功(is_ready_ == true 才可调用 Detect)
    bool IsReady() const { return is_ready_; }

    /// 当前模型类型
    ModelType model_type() const { return model_type_; }

    // 由 config 配置的超参数(公开, 便于节点从参数设置)
    float score_threshold_ = 0.35f;   ///< 置信度阈值(低于的直接丢弃)
    float nms_threshold_ = 0.45f;     ///< NMS 重叠阈值
    int input_size_ = 640;            ///< 网络输入边长(模型固定 640)

private:
    bool is_ready_ = false;   ///< 模型是否就绪
    ModelType model_type_ = ModelType::Detect;  ///< 模型类型
    cv::dnn::Net net_;        ///< OpenCV DNN 网络

    std::vector<cv::Mat> net_outputs_;  ///< 前向输出容器(复用, 避免反复分配)
    cv::Mat transposed_buffer_;         ///< 转置内存池(复用)

    /**
     * @brief 解码分割模型的掩膜(seg 模型专用)
     * @param coeffs      32 个掩膜系数(来自检测输出)
     * @param proto       原型掩膜 [1,32,160,160] 或 [32,160,160](第 2 个输出)
     * @param orig_img    原图(用于确定掩膜输出尺寸)
     * @param left, top   letterbox 左边/上边填充
     * @param scale       letterbox 等比缩放比
     * @param box         检测框(原图坐标), 掩膜会裁剪到框内
     * @return 原图大小的二值掩膜(CV_8UC1, 0/255, 框外为 0)
     */
    cv::Mat decodeMask(
        const std::vector<float> & coeffs, const cv::Mat & proto,
        const cv::Mat & orig_img, int left, int top, float scale,
        const cv::Rect & box);
};

}  // namespace culvert_core
