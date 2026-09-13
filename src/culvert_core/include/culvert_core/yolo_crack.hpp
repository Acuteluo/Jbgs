// Copyright (c) 2024, culvert_core contributors
//
// 文件: yolo_crack.hpp
// 作用: 裂缝检测结果类(仿照旧项目 yolo_armor, 但面向单类裂缝检测)。
//
// 对应旧项目 yolo_armor 的职责:
//   - 旧 yolo_armor 负责存储"一个装甲板"的检测结果 + PnP 解算 + 画框;
//   - 这里 yolo_crack 负责存储"一个裂缝"的检测结果 + 画框。
//   (裂缝没有装甲板那种 3D 姿态/PnP 需求, 因此省掉 PnP 部分。)
//
// CrackObject(定义在 yolo_detector.hpp) 是模型解析出的原始结果;
// YoloCrack 在其之上提供面向显示的封装: 画框、标签、统计信息。

#pragma once

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "culvert_core/yolo_detector.hpp"

namespace culvert_core
{

/**
 * @brief 单个裂缝的显示封装: 存检测结果 + 提供画框函数
 */
class YoloCrack
{
public:
    /// 由模型结果构造(记录 box 与置信度)
    YoloCrack(const cv::Rect & box, float confidence, int class_id);

    /// 设置分割掩膜(seg 模型才有; 检测模型不调用则只画框)
    void SetMask(const cv::Mat & mask) { mask_ = mask; }

    /// 在图上画裂缝框与置信度标签(有掩膜时额外描边)
    void Draw(cv::Mat & img_show) const;

    // ---- 检测结果(只读) ----
    cv::Rect Box() const { return box_; }
    float Confidence() const { return confidence_; }
    int ClassId() const { return class_id_; }

private:
    cv::Rect box_;        ///< 裂缝矩形框(原图坐标)
    float confidence_;    ///< 置信度(0~1)
    int class_id_;        ///< 类别 id(0 = Crack)
    cv::Mat mask_;        ///< 分割掩膜(原图大小, 0/255); 空 = 无掩膜
};

/// 工具函数: 把一帧图像 + 检测结果画到一起(包含顶部统计栏),
/// 供检测线程直接调用后 imshow。
void DrawCrackResults(
    cv::Mat & img_show,
    const std::vector<CrackObject> & results,
    double infer_ms,
    double fps);

}  // namespace culvert_core
