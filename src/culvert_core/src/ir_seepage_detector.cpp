// Copyright (c) 2024, culvert_core contributors
//
// 文件: ir_seepage_detector.cpp
// 作用: 红外渗水检测器实现(合并自 Culvert-Visual-Inspection-main,
//       算法与原 demo/src/ir_seepage_detector.cpp 一致)。

#include "culvert_core/ir_seepage_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace culvert_core
{

IrSeepageDetector::IrSeepageDetector(IrSeepageParams params)
{
    setParams(params);
}

void IrSeepageDetector::setParams(const IrSeepageParams & params)
{
    params_ = params;
    params_.ksize = oddKernel(params_.ksize);
    params_.morph_ksize = oddKernel(params_.morph_ksize);
    params_.morph_close_ksize = oddKernel(params_.morph_close_ksize);
    params_.min_area_ratio = std::clamp(params_.min_area_ratio, 0.0, 1.0);
    if (params_.clahe_grid < 2)
    {
        params_.clahe_grid = 2;
    }
    // 时间一致性参数钳位: N>=1, 1<=M<=N(M>N 永远无法确认, 直接收敛)
    if (params_.temporal_window < 1)
    {
        params_.temporal_window = 1;
    }
    if (params_.temporal_min_hits < 1)
    {
        params_.temporal_min_hits = 1;
    }
    if (params_.temporal_min_hits > params_.temporal_window)
    {
        params_.temporal_min_hits = params_.temporal_window;
    }
    // 参数整定会改变滑窗语义, 旧历史一律作废
    reset();
}

void IrSeepageDetector::reset()
{
    hit_buf_.clear();
    hit_buf_.shrink_to_fit();
    hit_count_.release();
    state_size_ = cv::Size();
    hit_head_ = 0;
    hit_fill_ = 0;
}

int IrSeepageDetector::oddKernel(int k)
{
    if (k < 3)
    {
        k = 3;
    }
    if ((k & 1) == 0)
    {
        ++k;
    }
    return k;
}

int IrSeepageDetector::clampKernel(int k, const cv::Size & image_size)
{
    const int limit = std::min(image_size.width, image_size.height);
    if (limit < 3)
    {
        return 3;
    }
    int max_odd = limit - 1;
    if ((max_odd & 1) == 0)
    {
        --max_odd;
    }
    if (max_odd < 3)
    {
        max_odd = 3;
    }
    return std::min(k, max_odd);
}

IrSeepageResult IrSeepageDetector::detect(const cv::Mat & frame)
{
    IrSeepageResult out;
    if (frame.empty())
    {
        return out;
    }

    // ---- 1. 统一转 8bit 单通道灰度(白热图) ----
    cv::Mat gray;
    if (frame.channels() == 3)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else if (frame.channels() == 4)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    }
    else
    {
        gray = frame;
    }
    if (gray.type() != CV_8UC1)
    {
        gray.convertTo(gray, CV_8UC1);
    }

    // ---- 2. 可选 CLAHE(低对比涵洞放大局部反差, 不改变"湿区更暗"符号) ----
    cv::Mat enhanced = gray;
    if (params_.enable_clahe)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
            params_.clahe_clip,
            cv::Size(params_.clahe_grid, params_.clahe_grid));
        clahe->apply(gray, enhanced);
    }
    out.enhanced = enhanced;

    // ---- 3. 浮点提升 + 超大核高斯 -> 理论干燥背景场 B ----
    // 必须浮点相减: 8bit 减法下溢会把负差抹成 0, 冷斑信息丢失。
    cv::Mat src_f;
    enhanced.convertTo(src_f, CV_32F);

    const int ksize = clampKernel(params_.ksize, src_f.size());
    cv::GaussianBlur(src_f, out.background, cv::Size(ksize, ksize), 0.0);

    // ---- 4. Diff = B - I: 渗水在白热图中偏暗 -> Diff 为正 ----
    cv::subtract(out.background, src_f, out.diff);

    // ---- 5. 单帧命中图: Diff > 阈值 -> 0/255 二值 ----
    cv::Mat bin;
    cv::threshold(out.diff, bin, params_.diff_thresh, 255.0, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_8U);

    // ---- 6. 时间一致性确认(像素级): 最近 N 帧滑窗内命中 >= M 才保留 ----
    // 环形缓冲增量维护命中计数: 覆盖最旧帧前先减去它的贡献, 再加新帧,
    // 避免每帧对整窗求和。渗水斑在停点画面中近似静止, 真实渗水会持续
    // 命中; 单帧噪声/JPEG 块效应/AGC 抖动难以连续命中, 被自然滤除。
    cv::Mat confirmed;
    if (params_.temporal_window <= 1)
    {
        // 单帧模式: 关闭滑窗, 与无时间一致性的旧行为完全一致
        confirmed = bin;
        reset();
    }
    else
    {
        // 首帧或帧尺寸变化 => 状态按新尺寸重建(旧历史无法复用)
        if (bin.size() != state_size_)
        {
            reset();
            state_size_ = bin.size();
            hit_buf_.assign(params_.temporal_window, cv::Mat());
            hit_count_ = cv::Mat::zeros(bin.size(), CV_16U);
        }
        // 命中图归一为 0/1, 便于累加
        cv::Mat hit01;
        bin.convertTo(hit01, CV_8U, 1.0 / 255.0);

        if (hit_fill_ == hit_buf_.size())
        {
            // 窗口已满: 先移除被覆盖的最旧帧的贡献
            cv::subtract(hit_count_, hit_buf_[hit_head_], hit_count_,
                         cv::noArray(), CV_16U);
        }
        else
        {
            ++hit_fill_;
        }
        hit_buf_[hit_head_] = hit01;
        cv::add(hit_count_, hit01, hit_count_, cv::noArray(), CV_16U);
        hit_head_ = (hit_head_ + 1) % hit_buf_.size();

        // 确认掩膜: 命中次数 >= min_hits(经 32F 中转做阈值, 16U 兼容性稳)
        cv::Mat count_f;
        hit_count_.convertTo(count_f, CV_32F);
        cv::threshold(count_f, confirmed,
                      static_cast<double>(params_.temporal_min_hits) - 0.5,
                      255.0, cv::THRESH_BINARY);
        confirmed.convertTo(confirmed, CV_8U);
        out.hit_count = hit_count_.clone();
    }

    // ---- 7. 形态学(开运算去单点噪声, 闭运算连片) ----
    const cv::Mat open_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(params_.morph_ksize, params_.morph_ksize));
    cv::morphologyEx(confirmed, out.mask, cv::MORPH_OPEN, open_kernel);

    const int close_k = clampKernel(params_.morph_close_ksize, out.mask.size());
    const cv::Mat close_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(close_k, close_k));
    cv::morphologyEx(out.mask, out.mask, cv::MORPH_CLOSE, close_kernel);

    // ---- 8. 连通域 -> 区域候选(小面积/碎轮廓过滤) ----
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(out.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<IrSeepageRegion> candidates;
    candidates.reserve(contours.size());
    double max_area = 0.0;
    for (const auto & contour : contours)
    {
        const double area = cv::contourArea(contour);
        if (area < params_.min_area_px)
        {
            continue;
        }

        IrSeepageRegion region;
        region.contour = contour;
        region.bbox = cv::boundingRect(contour);
        region.area_px = area;

        // 区域内平均变暗量: 越大说明热对比越可靠(置信辅助量)
        cv::Mat region_mask = cv::Mat::zeros(out.mask.size(), CV_8UC1);
        cv::drawContours(region_mask, std::vector<std::vector<cv::Point>>{contour},
                         0, cv::Scalar(255), cv::FILLED);
        region.mean_diff = static_cast<float>(cv::mean(out.diff, region_mask)[0]);
        // 区域内平均命中帧数: 稳定渗水应接近滑窗长度 N(置信辅助量)
        if (!out.hit_count.empty())
        {
            region.mean_hits =
                static_cast<float>(cv::mean(out.hit_count, region_mask)[0]);
        }
        max_area = std::max(max_area, area);
        candidates.push_back(std::move(region));
    }

    out.regions.clear();
    out.regions.reserve(candidates.size());
    const double area_cut = max_area * params_.min_area_ratio;
    for (auto & region : candidates)
    {
        if (region.area_px + 1e-6 < area_cut)
        {
            continue;
        }
        out.regions.push_back(std::move(region));
    }

    // 最终掩膜只保留保留区域(与输出 regions 一致, 供回溯/量化用)
    out.mask = cv::Mat::zeros(bin.size(), CV_8UC1);
    for (const auto & region : out.regions)
    {
        cv::drawContours(out.mask, std::vector<std::vector<cv::Point>>{region.contour},
                         0, cv::Scalar(255), cv::FILLED);
    }

    return out;
}

void IrSeepageDetector::drawOverlay(
    cv::Mat & bgr, const IrSeepageResult & result, const cv::Scalar & color)
{
    if (bgr.empty())
    {
        return;
    }
    for (const auto & region : result.regions)
    {
        if (!region.contour.empty())
        {
            cv::drawContours(bgr, std::vector<std::vector<cv::Point>>{region.contour},
                             0, color, 2, cv::LINE_AA);
        }
        std::string label = cv::format("dT=%.1f A=%.0f H=%.1f",
                                       region.mean_diff, region.area_px,
                                       region.mean_hits);
        cv::Point text(region.bbox.x, std::max(region.bbox.y - 6, 14));
        cv::putText(bgr, label, text, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 0, 0), 3);
        cv::putText(bgr, label, text, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
    }
}

}  // namespace culvert_core
