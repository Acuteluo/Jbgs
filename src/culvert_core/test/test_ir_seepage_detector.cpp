// Copyright (c) 2024, culvert_core contributors
//
// 文件: test_ir_seepage_detector.cpp
// 作用: IrSeepageDetector 时间一致性(滑窗确认)单元测试。
//
// 构图约定(与 ir_camera_node 的模拟热像同思路, 但完全确定性):
//   垂直渐变底(110~150) + 高斯冷斑(幅值 -50) => 冷斑处 Diff ≈ 40+
//   远超 diff_thresh=15, 干净背景 Diff ≈ 0。不含随机噪声, 断言稳定。

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include "culvert_core/ir_seepage_detector.hpp"

using culvert_core::IrSeepageDetector;
using culvert_core::IrSeepageParams;

namespace
{

// 垂直渐变"干燥背景"帧(与模拟热像的墙面温度梯度一致)
cv::Mat MakeGradientFrame(int w, int h)
{
    cv::Mat f(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y)
    {
        f.row(y).setTo(
            cv::saturate_cast<uchar>(110 + y * 40 / std::max(h, 1)));
    }
    return f;
}

// 在帧上叠加高斯冷斑(模拟渗水, 白热图上偏暗)
void AddColdBlob(
    cv::Mat & f, const cv::Point2f & center, float sigma, float amplitude)
{
    const int r = static_cast<int>(sigma * 3.0f);
    const int x0 = std::max(0, static_cast<int>(center.x) - r);
    const int y0 = std::max(0, static_cast<int>(center.y) - r);
    const int x1 = std::min(f.cols, static_cast<int>(center.x) + r);
    const int y1 = std::min(f.rows, static_cast<int>(center.y) + r);
    for (int y = y0; y < y1; ++y)
    {
        for (int x = x0; x < x1; ++x)
        {
            const float dx = x - center.x;
            const float dy = y - center.y;
            const float g = std::exp(
                -(dx * dx + dy * dy) / (2.0f * sigma * sigma));
            const int v = f.at<uchar>(y, x) +
                          static_cast<int>(amplitude * g);
            f.at<uchar>(y, x) = cv::saturate_cast<uchar>(v);
        }
    }
}

// 单冷斑帧: 64x48, 斑心 (32,24), sigma=3, 幅值 -50
cv::Mat BlobFrame()
{
    cv::Mat f = MakeGradientFrame(64, 48);
    AddColdBlob(f, cv::Point2f(32.f, 24.f), 3.0f, -50.0f);
    return f;
}

IrSeepageParams TestParams(int window, int min_hits)
{
    IrSeepageParams p;
    p.ksize = 31;               // 核尺度须大于冷斑直径的 1.5 倍, 否则冷斑
                                // 被抹进背景场(中空), diff 峰值贴阈值
    p.diff_thresh = 15.f;
    p.morph_ksize = 3;
    p.morph_close_ksize = 5;
    p.min_area_px = 20.0;
    p.min_area_ratio = 0.0;     // 不过滤次生区域, 只验证时间逻辑
    p.enable_clahe = false;
    p.temporal_window = window;
    p.temporal_min_hits = min_hits;
    return p;
}

}  // namespace

// 偶发单帧命中(冷物体飞过/JPEG 块效应)必须被滑窗滤除
TEST(IrSeepageTemporal, RejectsSingleFrameBurst)
{
    IrSeepageDetector det(TestParams(5, 3));
    const cv::Mat clean = MakeGradientFrame(64, 48);

    // 第 1 帧出现冷斑, 之后 4 帧消失 => 窗口内命中最多 1 < 3, 全程无输出
    auto r = det.detect(BlobFrame());
    EXPECT_EQ(r.regions.size(), 0u);
    for (int i = 0; i < 4; ++i)
    {
        r = det.detect(clean);
        EXPECT_EQ(r.regions.size(), 0u) << "clean frame " << i;
    }
}

// 持续存在的渗水斑在累计 min_hits 帧后被确认
TEST(IrSeepageTemporal, ConfirmsPersistentBlob)
{
    IrSeepageDetector det(TestParams(5, 3));

    // 预热期(前 M-1 帧)无输出, 第 M 帧起确认
    auto r = det.detect(BlobFrame());
    EXPECT_EQ(r.regions.size(), 0u);
    r = det.detect(BlobFrame());
    EXPECT_EQ(r.regions.size(), 0u);
    r = det.detect(BlobFrame());
    ASSERT_GE(r.regions.size(), 1u);
    // 确认后命中计数应接近满窗(斑未动): mean_hits >= M
    EXPECT_GE(r.regions[0].mean_hits,
              static_cast<float>(det.params().temporal_min_hits) - 0.5f);
}

// 斑消失后, 已确认区域按滑窗拖尾保留 N-M 帧, 随旧命中滑出而消失
TEST(IrSeepageTemporal, BlobDisappearsWhenWindowSlidesOut)
{
    IrSeepageDetector det(TestParams(5, 3));
    const cv::Mat clean = MakeGradientFrame(64, 48);

    for (int i = 0; i < 3; ++i)
    {
        det.detect(BlobFrame());   // 确认
    }
    // 拖尾期: 窗口里还剩 3 个历史命中 => 仍确认(N-M=2 帧)
    auto r = det.detect(clean);
    EXPECT_GE(r.regions.size(), 1u);
    r = det.detect(clean);
    EXPECT_GE(r.regions.size(), 1u);
    // 第 3 帧起旧命中不足以维持 M => 区域消失
    r = det.detect(clean);
    EXPECT_EQ(r.regions.size(), 0u);
    r = det.detect(clean);
    EXPECT_EQ(r.regions.size(), 0u);
}

// reset() 清空滑窗: 已确认的历史不能在复位后"幸存"
TEST(IrSeepageTemporal, ResetClearsHistory)
{
    IrSeepageDetector det(TestParams(5, 3));

    for (int i = 0; i < 3; ++i)
    {
        det.detect(BlobFrame());   // 第 3 帧已确认
    }
    det.reset();

    // 复位后需重新累计: 前 M-1 帧无输出
    auto r = det.detect(BlobFrame());
    EXPECT_EQ(r.regions.size(), 0u);
    r = det.detect(BlobFrame());
    EXPECT_EQ(r.regions.size(), 0u);
    r = det.detect(BlobFrame());
    EXPECT_GE(r.regions.size(), 1u);
}

// temporal_window <= 1 关闭时间一致性: 单帧即出结果(旧行为回归)
TEST(IrSeepageTemporal, WindowOneDetectsImmediately)
{
    IrSeepageDetector det(TestParams(1, 1));
    const auto r = det.detect(BlobFrame());
    EXPECT_GE(r.regions.size(), 1u);
    // 单帧模式下 hit_count 为空(无滑窗状态)
    EXPECT_TRUE(r.hit_count.empty());
}

// 帧尺寸变化自动复位: 旧尺寸的确认历史不能带到新尺寸
TEST(IrSeepageTemporal, SizeChangeResetsState)
{
    IrSeepageDetector det(TestParams(5, 3));

    for (int i = 0; i < 3; ++i)
    {
        det.detect(BlobFrame());   // 64x48 已确认
    }

    // 换 80x60 帧: 状态重建, 单帧命中不足以确认
    cv::Mat big = MakeGradientFrame(80, 60);
    AddColdBlob(big, cv::Point2f(40.f, 30.f), 3.0f, -50.0f);
    const auto r = det.detect(big);
    EXPECT_EQ(r.regions.size(), 0u);
}

// min_hits > window 的非法配置被钳位成 M=N(不产生"永远无法确认"状态)
TEST(IrSeepageTemporal, MinHitsClampedToWindow)
{
    IrSeepageDetector det(TestParams(3, 99));
    EXPECT_EQ(det.params().temporal_min_hits, 3);

    for (int i = 0; i < 3; ++i)
    {
        const auto r = det.detect(BlobFrame());
        if (i == 2)
        {
            EXPECT_GE(r.regions.size(), 1u);
        }
        else
        {
            EXPECT_EQ(r.regions.size(), 0u);
        }
    }
}
