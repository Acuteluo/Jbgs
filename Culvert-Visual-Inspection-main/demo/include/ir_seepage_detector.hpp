#pragma once

#include <opencv2/core.hpp>
#include <vector>

// 红外白热灰度流上的局部差分渗水检测（无厂家 SDK、无模型）。
// Diff = 大核高斯背景 − 原图，正值表示相对邻域变暗。

struct IrSeepageParams {
    int ksize = 71;              // 高斯核边长，奇数；须 > 最大水斑直径 * 1.5
    float diff_thresh = 15.f;    // 局部灰阶差阈值：10~15 高灵敏，18~25 抗噪
    int morph_ksize = 3;         // 开运算核，奇数，去单点噪声
    int morph_close_ksize = 21;  // 闭运算核，把近处碎斑连成整片
    double min_area_px = 120.0;  // 小于该面积的连通域视为噪声
    double min_area_ratio = 0.3; // 相对最大连通域的面积比，更小的碎轮廓丢掉
    bool enable_clahe = false;   // 低对比涵洞可打开以放大局部反差
    double clahe_clip = 2.0;
    int clahe_grid = 8;
};

struct IrSeepageRegion {
    cv::Rect bbox;
    std::vector<cv::Point> contour;
    double area_px = 0.0;
    float mean_diff = 0.f;       // 区域内平均变暗量，越大热对比越可靠
};

struct IrSeepageResult {
    std::vector<IrSeepageRegion> regions;
    cv::Mat mask;        // CV_8UC1，渗水 = 255
    cv::Mat background;  // CV_32F 理论干燥背景
    cv::Mat diff;        // CV_32F，B - I
    cv::Mat enhanced;    // 送入差分前的 8U 图（含可选 CLAHE）
};

class IrSeepageDetector {
public:
    explicit IrSeepageDetector(IrSeepageParams params = {});

    void setParams(const IrSeepageParams& params);
    const IrSeepageParams& params() const { return params_; }

    IrSeepageResult detect(const cv::Mat& frame) const;

    static void drawOverlay(cv::Mat& bgr, const IrSeepageResult& result,
                            const cv::Scalar& color = cv::Scalar(0, 255, 255));

private:
    IrSeepageParams params_;

    static int oddKernel(int k);
    static int clampKernel(int k, const cv::Size& image_size);
};
