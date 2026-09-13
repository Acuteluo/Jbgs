// Copyright (c) 2024, culvert_core contributors
//
// 文件: ir_seepage_detector.hpp
// 作用: 红外白热灰度流上的局部差分渗水检测(无厂家 SDK、无模型)。
//
// 来源说明:
//   本文件合并自 Culvert-Visual-Inspection-main 项目
//   (demo/include/ir_seepage_detector.hpp, 厂家 UVC 机芯驱动方案改写),
//   仅包一层 culvert_core 命名空间, 算法逻辑与原实现保持一致。
//
// 算法原理(详见 Culvert-Visual-Inspection-main/docs/红外传统渗水检测方案.md):
//   Diff = 大核高斯背景场 B - 原图 I, 正值表示当前点相对邻域变暗。
//   白热模式下渗水因蒸发吸热呈局部冷斑, 故 Diff 显著为正;
//   AGC 带来的全图整体拉亮/压暗在 B 与 I 中同时出现, 相减后抵消,
//   因此不依赖全局绝对灰度阈值。
//   Diff -> 二值化 -> 开运算去噪 -> 闭运算连片 -> 连通域 -> BBox/Mask。

#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace culvert_core
{

/// 渗水检测参数(默认值与原 demo 一致; 256x192 机芯会自动钳位核尺寸)。
struct IrSeepageParams
{
    int ksize = 71;              ///< 高斯核边长, 奇数; 须 > 最大水斑直径 * 1.5
    float diff_thresh = 15.f;    ///< 局部灰阶差阈值: 10~15 高灵敏, 18~25 抗噪
    int morph_ksize = 3;         ///< 开运算核, 奇数, 去单点噪声
    int morph_close_ksize = 21;  ///< 闭运算核, 把近处碎斑连成整片
    double min_area_px = 120.0;  ///< 小于该面积的连通域视为噪声
    double min_area_ratio = 0.3; ///< 相对最大连通域的面积比, 更小的碎轮廓丢掉
    bool enable_clahe = false;   ///< 低对比涵洞可打开以放大局部反差
    double clahe_clip = 2.0;
    int clahe_grid = 8;
};

/// 一个渗水连通域。
struct IrSeepageRegion
{
    cv::Rect bbox;                  ///< 轴对齐外接矩形(原图坐标)
    std::vector<cv::Point> contour; ///< 外轮廓(用于描边标注)
    double area_px = 0.0;           ///< 连通域面积(像素)
    float mean_diff = 0.f;          ///< 区域内平均变暗量, 越大热对比越可靠
};

/// 单帧检测结果。
struct IrSeepageResult
{
    std::vector<IrSeepageRegion> regions;  ///< 保留的渗水区域
    cv::Mat mask;        ///< CV_8UC1, 渗水 = 255
    cv::Mat background;  ///< CV_32F 理论干燥背景场
    cv::Mat diff;        ///< CV_32F, B - I
    cv::Mat enhanced;    ///< 送入差分前的 8U 图(含可选 CLAHE)
};

/// 红外渗水检测器: 单帧检测, 无 GUI、无相机依赖, 可独立单测。
class IrSeepageDetector
{
public:
    explicit IrSeepageDetector(IrSeepageParams params = {});

    void setParams(const IrSeepageParams & params);
    const IrSeepageParams & params() const { return params_; }

    /// 检测一帧(输入 BGR/灰度均可, 内部统一转 8bit 灰度)。
    /// 线程约定: 本方法只读内部参数、只写局部输出, 在单个检测线程内
    /// 串行调用(与 core_node 的 IrDetectLoop 配合), 无需加锁。
    IrSeepageResult detect(const cv::Mat & frame) const;

    /// 在 BGR 图上画渗水标注(黄色轮廓 + dT/面积标签)。
    static void drawOverlay(
        cv::Mat & bgr, const IrSeepageResult & result,
        const cv::Scalar & color = cv::Scalar(0, 255, 255));

private:
    IrSeepageParams params_;

    /// 把核尺寸规整为 >=3 的奇数。
    static int oddKernel(int k);
    /// 把核尺寸钳位到图像短边以内的最大奇数(防止边界效应抹平全图)。
    static int clampKernel(int k, const cv::Size & image_size);
};

}  // namespace culvert_core
