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
//   Diff -> 二值化 -> [时间一致性确认: 最近 N 帧滑窗内命中 >= M 才保留]
//        -> 开运算去噪 -> 闭运算连片 -> 连通域 -> BBox/Mask。
//   时间一致性为像素级确认, 不做跨帧区域匹配: 停点拍摄时渗水斑在画面
//   中近似静止, 而单帧噪声/JPEG 块效应/AGC 抖动难以连续命中, 被自然滤除;
//   temporal_window <= 1 时关闭, 退化为纯单帧检测。

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
    // ---- 时间一致性(抗单帧误报) ----
    int temporal_window = 1;     ///< 滑窗长度 N(帧); <=1 = 关闭(参考工程单帧行为)
    int temporal_min_hits = 1;   ///< 窗口内至少命中帧数 M; 自动钳位到 <= N。
                                 ///< 代价: 前 M-1 帧为预热期, 不输出区域
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
    float mean_hits = 0.f;          ///< 区域内平均命中帧数(窗口内, 0~N),
                                    ///< 稳定渗水应接近 N; 偏低说明刚出现或快消失
};

/// 单帧检测结果。
struct IrSeepageResult
{
    std::vector<IrSeepageRegion> regions;  ///< 保留的渗水区域
    cv::Mat mask;        ///< CV_8UC1, 渗水 = 255
    cv::Mat background;  ///< CV_32F 理论干燥背景场
    cv::Mat diff;        ///< CV_32F, B - I
    cv::Mat enhanced;    ///< 送入差分前的 8U 图(含可选 CLAHE)
    cv::Mat hit_count;   ///< CV_16U, 滑窗内各像素命中帧数(单帧模式为空)
};

/// 红外渗水检测器: 单帧差分 + 像素级时间一致性确认, 无 GUI、无相机依赖,
/// 可独立单测。

/// 红外渗水检测器: 单帧检测, 无 GUI、无相机依赖, 可独立单测。
class IrSeepageDetector
{
public:
    explicit IrSeepageDetector(IrSeepageParams params = {});

    void setParams(const IrSeepageParams & params);
    const IrSeepageParams & params() const { return params_; }

    /// 检测一帧(输入 BGR/灰度均可, 内部统一转 8bit 灰度)。
    /// 时间一致性开启时本方法带内部状态(滑窗环形缓冲), 且不再是 const:
    /// 连续送入同一相机的时间序列帧才有效。线程约定: 在单个检测线程内
    /// 串行调用(与 core_node 的 IrDetectLoop 配合), 无需加锁;
    /// 断流恢复/场景切换请先调用 reset() 清空滑窗历史。
    IrSeepageResult detect(const cv::Mat & frame);

    /// 清空时间一致性滑窗历史(帧尺寸变化时 detect 内部也会自动复位)。
    void reset();

    /// 在 BGR 图上画渗水标注(黄色轮廓 + dT/面积标签)。
    static void drawOverlay(
        cv::Mat & bgr, const IrSeepageResult & result,
        const cv::Scalar & color = cv::Scalar(0, 255, 255));

private:
    IrSeepageParams params_;

    // ---- 时间一致性状态(仅 detect/reset 内串行读写) ----
    std::vector<cv::Mat> hit_buf_;  ///< 最近 N 帧命中掩膜(0/1, CV_8U)环形缓冲
    cv::Mat hit_count_;             ///< 窗口内命中次数累加(CV_16U, 增量维护)
    cv::Size state_size_{};         ///< 滑窗状态对应的帧尺寸(变尺寸即复位)
    size_t hit_head_ = 0;           ///< 环形缓冲下一写入位置
    size_t hit_fill_ = 0;           ///< 已填充帧数(0~N)

    /// 把核尺寸规整为 >=3 的奇数。
    static int oddKernel(int k);
    /// 把核尺寸钳位到图像短边以内的最大奇数(防止边界效应抹平全图)。
    static int clampKernel(int k, const cv::Size & image_size);
};

}  // namespace culvert_core
