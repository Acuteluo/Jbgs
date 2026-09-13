#include "ir_seepage_detector.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>

IrSeepageDetector::IrSeepageDetector(IrSeepageParams params) {
    setParams(params);
}

void IrSeepageDetector::setParams(const IrSeepageParams& params) {
    params_ = params;
    params_.ksize = oddKernel(params_.ksize);
    params_.morph_ksize = oddKernel(params_.morph_ksize);
    params_.morph_close_ksize = oddKernel(params_.morph_close_ksize);
    params_.min_area_ratio = std::clamp(params_.min_area_ratio, 0.0, 1.0);
    if (params_.clahe_grid < 2) {
        params_.clahe_grid = 2;
    }
}

int IrSeepageDetector::oddKernel(int k) {
    if (k < 3) {
        k = 3;
    }
    if ((k & 1) == 0) {
        ++k;
    }
    return k;
}

int IrSeepageDetector::clampKernel(int k, const cv::Size& image_size) {
    const int limit = std::min(image_size.width, image_size.height);
    if (limit < 3) {
        return 3;
    }
    int max_odd = limit - 1;
    if ((max_odd & 1) == 0) {
        --max_odd;
    }
    if (max_odd < 3) {
        max_odd = 3;
    }
    return std::min(k, max_odd);
}

IrSeepageResult IrSeepageDetector::detect(const cv::Mat& frame) const {
    IrSeepageResult out;
    if (frame.empty()) {
        return out;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame;
    }
    if (gray.type() != CV_8UC1) {
        gray.convertTo(gray, CV_8UC1);
    }

    cv::Mat enhanced = gray;
    if (params_.enable_clahe) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
            params_.clahe_clip,
            cv::Size(params_.clahe_grid, params_.clahe_grid));
        clahe->apply(gray, enhanced);
    }
    out.enhanced = enhanced;

    cv::Mat src_f;
    enhanced.convertTo(src_f, CV_32F);

    const int ksize = clampKernel(params_.ksize, src_f.size());
    cv::GaussianBlur(src_f, out.background, cv::Size(ksize, ksize), 0.0);

    // 渗水在白热图中偏暗：背景亮、原图暗 → Diff 为正。
    cv::subtract(out.background, src_f, out.diff);

    cv::Mat bin;
    cv::threshold(out.diff, bin, params_.diff_thresh, 255.0, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_8U);

    const cv::Mat open_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(params_.morph_ksize, params_.morph_ksize));
    cv::morphologyEx(bin, out.mask, cv::MORPH_OPEN, open_kernel);

    const int close_k = clampKernel(params_.morph_close_ksize, out.mask.size());
    const cv::Mat close_kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(close_k, close_k));
    cv::morphologyEx(out.mask, out.mask, cv::MORPH_CLOSE, close_kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(out.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<IrSeepageRegion> candidates;
    candidates.reserve(contours.size());
    double max_area = 0.0;
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < params_.min_area_px) {
            continue;
        }

        IrSeepageRegion region;
        region.contour = contour;
        region.bbox = cv::boundingRect(contour);
        region.area_px = area;

        cv::Mat region_mask = cv::Mat::zeros(out.mask.size(), CV_8UC1);
        cv::drawContours(region_mask, std::vector<std::vector<cv::Point>>{contour},
                         0, cv::Scalar(255), cv::FILLED);
        region.mean_diff = static_cast<float>(cv::mean(out.diff, region_mask)[0]);
        max_area = std::max(max_area, area);
        candidates.push_back(std::move(region));
    }

    out.regions.clear();
    out.regions.reserve(candidates.size());
    const double area_cut = max_area * params_.min_area_ratio;
    for (auto& region : candidates) {
        if (region.area_px + 1e-6 < area_cut) {
            continue;
        }
        out.regions.push_back(std::move(region));
    }

    out.mask = cv::Mat::zeros(bin.size(), CV_8UC1);
    for (const auto& region : out.regions) {
        cv::drawContours(out.mask, std::vector<std::vector<cv::Point>>{region.contour},
                         0, cv::Scalar(255), cv::FILLED);
    }

    return out;
}

void IrSeepageDetector::drawOverlay(cv::Mat& bgr, const IrSeepageResult& result,
                                    const cv::Scalar& color) {
    if (bgr.empty()) {
        return;
    }
    for (const auto& region : result.regions) {
        if (!region.contour.empty()) {
            cv::drawContours(bgr, std::vector<std::vector<cv::Point>>{region.contour},
                             0, color, 2, cv::LINE_AA);
        }
        std::string label = cv::format("dT=%.1f A=%.0f", region.mean_diff, region.area_px);
        cv::Point text(region.bbox.x, std::max(region.bbox.y - 6, 14));
        cv::putText(bgr, label, text, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 0, 0), 3);
        cv::putText(bgr, label, text, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
    }
}
