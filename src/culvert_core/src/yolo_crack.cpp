// Copyright (c) 2024, culvert_core contributors
//
// 文件: yolo_crack.cpp
// 作用: 裂缝检测结果类实现(画框、掩膜描边、统计信息绘制)。

#include "culvert_core/yolo_crack.hpp"

#include <algorithm>
#include <string>

namespace culvert_core
{

YoloCrack::YoloCrack(const cv::Rect & box, float confidence, int class_id)
: box_(box),
  confidence_(confidence),
  class_id_(class_id)
{
}

void YoloCrack::Draw(cv::Mat & img_show) const
{
    // 1. 若有分割掩膜: 先描边(把裂缝轮廓画出来)。
    //    掩膜是原图大小的 0/255 二值图, 提取轮廓后画青色粗线。
    if (!mask_.empty())
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(img_show, contours, -1, cv::Scalar(255, 200, 0), 2);
    }

    // 2. 画裂缝矩形框(红色)。
    //    线宽 3: 贴边/小框在压缩后仍清晰可见。
    cv::rectangle(img_show, box_, cv::Scalar(0, 0, 255), 3);

    // 3. 置信度标签(绿色)。默认画在框上方; 但若框贴近顶部, 标签会被
    //    顶部白色信息栏(高 24px)或画布上沿遮住, 此时改放到框下方;
    //    若框几乎占满整幅图(上下都放不下), 则画在框内顶部 ——
    //    保证任何位置的框, 标签都清晰可见。
    const std::string label = "Crack " + std::to_string(
        static_cast<int>(confidence_ * 100)) + "%";
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double font_scale = 0.6;
    const int thickness = 2;

    // 用 getTextSize 精确量出文本高度, 避免拍脑袋估算
    int baseline = 0;
    const cv::Size text_size =
        cv::getTextSize(label, font, font_scale, thickness, &baseline);
    const int text_h = text_size.height + baseline + 6;   // 文本总占用高度

    // x: 至少 4px, 且不能超出右边界
    int x = box_.x;
    const int x_max = img_show.cols - text_size.width - 4;
    x = std::max(4, std::min(x, x_max));

    constexpr int kTopBarHeight = 24;   // 与 DrawCrackResults 顶部信息栏高度一致
    // y 是文字基线(文字占据 [y - text_h, y])。先按"放框上方"算
    int y = box_.y - 8;
    if (y - text_h < kTopBarHeight)
    {
        // 上方会被信息栏遮住 -> 放框下方(留 4px 间距, 不贴住红框边线)
        y = box_.y + box_.height + text_h + 4;
        if (y > img_show.rows - 6)
        {
            // 下方也越界(框几乎占满整幅图) -> 画在框内顶部,
            // 但仍要保证不被顶部信息栏遮住
            y = std::max(box_.y + text_h + 4, kTopBarHeight + text_h);
            y = std::min(y, img_show.rows - 6);
        }
    }
    cv::putText(img_show, label, cv::Point(x, y), font, font_scale,
                cv::Scalar(0, 255, 0), thickness);
}

void DrawCrackResults(
    cv::Mat & img_show,
    const std::vector<CrackObject> & results,
    double infer_ms,
    double fps)
{
    // 1. 逐个画框(检测模型只画框; 分割模型额外描边)
    for (const auto & obj : results)
    {
        YoloCrack crack(obj.box, obj.confidence, obj.class_id);
        crack.SetMask(obj.mask);
        crack.Draw(img_show);
    }

    // 2. 顶部信息栏(白底黑字, 便于查看)
    const std::string info =
        "cracks=" + std::to_string(results.size()) +
        "  infer=" + std::to_string(static_cast<int>(infer_ms)) + "ms" +
        "  fps=" + std::to_string(static_cast<int>(fps));

    cv::rectangle(img_show, cv::Rect(0, 0, img_show.cols, 24),
                  cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(img_show, info, cv::Point(8, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1);
}

}  // namespace culvert_core
