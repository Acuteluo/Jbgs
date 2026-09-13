// Copyright (c) 2024, culvert_core contributors
//
// 文件: yolo_detector.cpp
// 作用: 裂缝 YOLO 模型推理实现(基于 OpenCV DNN)。
//
// 详细注释见 yolo_detector.hpp。

#include "culvert_core/yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace culvert_core
{

YoloDetector::YoloDetector(
    const std::string & model_path, bool use_gpu, ModelType model_type)
: model_type_(model_type)
{
    // 强制开启底层数学指令集优化(唤醒 AVX2 等), 提高 CPU 推理速度
    cv::setUseOptimized(true);

    // 1. 检查模型文件是否存在, 防止 cv::dnn 加载时崩溃
    std::ifstream f(model_path.c_str());
    if (!f.good())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("YoloDetector"),
            "[YOLO] 找不到模型文件: %s", model_path.c_str());
        return;
    }

    try
    {
        // 2. 从 ONNX 加载网络结构与权重
        net_ = cv::dnn::readNetFromONNX(model_path);

        // 3. 选择推理引擎: CPU(OpenCV) 或 OpenVINO 加速
        if (use_gpu)
        {
            // 实测: 本机 OpenVINO 的 GPU(OpenCL) 后端跑此模型会输出异常
            // (置信度全部失真 -> 刷出几千个假框), 因此即使请求 GPU 也
            // 使用 OpenVINO 的 CPU 后端(输出正确, 且与 OpenCV CPU 相当)。
            // 若环境没有 OpenVINO, 构造会抛异常, 由下方 catch 回退 OpenCV。
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            RCLCPP_INFO(
                rclcpp::get_logger("YoloDetector"),
                "[YOLO] 引擎: OpenVINO CPU (GPU/OpenCL 后端本机不可靠, 已规避)");
        }
        else
        {
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            RCLCPP_INFO(
                rclcpp::get_logger("YoloDetector"),
                "[YOLO] 引擎: OpenCV CPU");
        }

        is_ready_ = true;
        RCLCPP_INFO(
            rclcpp::get_logger("YoloDetector"),
            "[YOLO] 模型加载成功(%s), OpenCV 线程数 = %d",
            model_type_ == ModelType::Seg ? "seg 分割" : "detect 检测",
            cv::getNumThreads());
    }
    catch (const cv::Exception & e)
    {
        // GPU 引擎初始化失败时, 回退到 CPU 重试一次
        if (use_gpu)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("YoloDetector"),
                "[YOLO] GPU 初始化失败(%s), 回退到 CPU", e.what());
            try
            {
                net_ = cv::dnn::readNetFromONNX(model_path);
                net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                is_ready_ = true;
                RCLCPP_INFO(
                    rclcpp::get_logger("YoloDetector"),
                    "[YOLO] 已回退: OpenCV CPU 模式");
            }
            catch (const cv::Exception & e2)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("YoloDetector"),
                    "[YOLO] CPU 回退也失败: %s", e2.what());
                is_ready_ = false;
            }
        }
        else
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("YoloDetector"),
                "[YOLO] DNN 初始化异常: %s", e.what());
            is_ready_ = false;
        }
    }
}

std::vector<CrackObject> YoloDetector::Detect(const cv::Mat & img)
{
    if (!is_ready_ || img.empty())
    {
        return {};
    }

    // ------------------------------------------------------------------
    // 1. 图像预处理: 与 Ultralytics 训练/推理一致的 LetterBox(letterbox)
    //    注意!! 必须复现训练时的预处理, 否则非方形图(如相机 2592x1944)
    //    会因"直接拉伸 vs 等比缩放+灰边"不同而导致: 检测精度下降、
    //    且坐标还原错位。训练用的 LetterBox(auto=False, scale_fill=False,
    //    center=True, padding=114, INTER_LINEAR)精确流程如下:
    //      r     = min(640/h, 640/w)      # 等比缩放比
    //      new_w = round(w*r), new_h = round(h*r)
    //      dw    = 640 - new_w            # 左右总填充
    //      dh    = 640 - new_h            # 上下总填充
    //      left  = round(dw/2 - 0.1)      # 居中: 左右/上下各垫一半
    //      top   = round(dh/2 - 0.1)
    //    然后 resize 到 (new_w, new_h), 再向四周补灰色(114) 边。
    // ------------------------------------------------------------------
    const int target = input_size_;
    const int h = img.rows;
    const int w = img.cols;

    const float r = std::min(
        static_cast<float>(target) / h,
        static_cast<float>(target) / w);
    const int new_w = static_cast<int>(std::round(w * r));
    const int new_h = static_cast<int>(std::round(h * r));
    const int dw = target - new_w;
    const int dh = target - new_h;
    const int left = static_cast<int>(std::round(dw * 0.5f - 0.1f));
    const int top = static_cast<int>(std::round(dh * 0.5f - 0.1f));

    // 等比缩放(双线性插值, 与训练一致)
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    // 灰色(114)填充到 640x640
    cv::Mat letterboxed(target, target, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterboxed(cv::Rect(left, top, new_w, new_h)));

    // 转 blob: /255 归一化, BGR->RGB(swapRB=true), 640x640
    cv::Mat blob = cv::dnn::blobFromImage(
        letterboxed, 1.0 / 255.0, cv::Size(target, target),
        cv::Scalar(0, 0, 0), true, false);

    // 2. 前向推理
    net_.setInput(blob);
    try
    {
        net_.forward(net_outputs_, net_.getUnconnectedOutLayersNames());
    }
    catch (const cv::Exception & e)
    {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("YoloDetector"), steady_clock, 1000,
            "[YOLO] 推理异常(本帧丢弃): %s", e.what());
        return {};
    }

    if (net_outputs_.empty())
    {
        return {};
    }

    // 3. 解析输出维度: 检测模型为 [1, 5, 8400],
    //    分割模型为 [1, 37, 8400](4框+1conf+32掩膜系数)
    cv::Mat output = net_outputs_[0];
    int rows = 0, cols = 0;
    const float * data_ptr = reinterpret_cast<const float *>(output.data);

    if (output.dims == 3)
    {
        rows = output.size[1];
        cols = output.size[2];
    }
    else if (output.dims == 2)
    {
        rows = output.size[0];
        cols = output.size[1];
    }
    else
    {
        return {};
    }

    // 输出可能是 [N, 8400](rows<cols) 或 [8400, N](rows>cols)。
    // 统一转成 [8400, N], 方便逐行遍历每个预测框。
    cv::Mat output_buffer;
    if (rows < cols)
    {
        cv::Mat temp(rows, cols, CV_32F, const_cast<float *>(data_ptr));
        cv::transpose(temp, transposed_buffer_);
        output_buffer = transposed_buffer_;
    }
    else
    {
        output_buffer = cv::Mat(rows, cols, CV_32F, const_cast<float *>(data_ptr));
    }

    const int num_boxes = output_buffer.rows;
    const int stride = output_buffer.cols;
    const float * data = reinterpret_cast<const float *>(output_buffer.data);

    // 每行的通道数: detect = 5(框+置信度), seg = 37(框+置信度+32掩膜系数)
    const int kNumMaskCoeffs = 32;
    const int conf_idx = 4;              ///< 置信度所在列
    const int mask_coeff_start = 5;      ///< seg 模型掩膜系数起始列

    // 分割模型的原型掩膜(第 2 个输出): [32, 160, 160]
    cv::Mat proto;
    if (model_type_ == ModelType::Seg && net_outputs_.size() >= 2)
    {
        proto = net_outputs_[1];
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    // seg 模型: 保存每个候选框的掩膜系数(原始输出行号 -> 系数)
    std::vector<std::vector<float>> mask_coeffs;

    // 4. 逐行解析: detect = [cx, cy, w, h, conf]
    //             seg    = [cx, cy, w, h, conf, coeff0..coeff31]
    //    注意: 本模型的 conf 列已是 sigmoid 后的概率, 不要再套 sigmoid!
    //    注意: 循环变量用 rr, 避免与外层缩放比 r(letterbox) 重名覆盖!
    for (int rr = 0; rr < num_boxes; ++rr)
    {
        const float * row = data + rr * stride;

        const float conf = row[conf_idx];
        if (conf < score_threshold_)
        {
            continue;
        }

        // 中心点/宽高都是相对"填充后的 640x640"的, 需做 letterbox 逆变换
        // 还原到原图坐标:
        //   x_orig = (x_640 - left) / r      y_orig = (y_640 - top) / r
        //   w_orig = w_640 / r               h_orig = h_640 / r
        const float cx = (row[0] - left) / r;
        const float cy = (row[1] - top) / r;
        const float w = row[2] / r;
        const float h = row[3] / r;

        const int x1 = static_cast<int>(cx - w * 0.5f);
        const int y1 = static_cast<int>(cy - h * 0.5f);

        boxes.emplace_back(x1, y1, static_cast<int>(w), static_cast<int>(h));
        confidences.push_back(conf);
        class_ids.push_back(0);   // 单类裂缝

        // seg 模型: 记录本候选框的 32 个掩膜系数
        if (model_type_ == ModelType::Seg && stride >= mask_coeff_start + kNumMaskCoeffs)
        {
            std::vector<float> coeffs(kNumMaskCoeffs);
            for (int c = 0; c < kNumMaskCoeffs; ++c)
            {
                coeffs[c] = row[mask_coeff_start + c];
            }
            mask_coeffs.push_back(std::move(coeffs));
        }
    }

    // 5. NMS: 去掉重叠的重复框
    std::vector<int> indices;
    if (!boxes.empty())
    {
        cv::dnn::NMSBoxes(
            boxes, confidences, score_threshold_, nms_threshold_, indices);
    }

    // 6. 组装最终结果(按置信度降序, 便于下游优先处理高置信裂缝)
    std::vector<CrackObject> results;
    results.reserve(indices.size());
    for (int idx : indices)
    {
        CrackObject obj;
        obj.box = boxes[idx];
        obj.confidence = confidences[idx];
        obj.class_id = class_ids[idx];

        // seg 模型: 解码裂缝掩膜(裁剪到检测框内, 避免框外出现描边)
        if (model_type_ == ModelType::Seg && !proto.empty())
        {
            obj.mask = decodeMask(
                mask_coeffs[idx], proto, img, left, top, r, obj.box);
        }

        results.push_back(obj);
    }

    std::sort(
        results.begin(), results.end(),
        [](const CrackObject & a, const CrackObject & b)
        {
            return a.confidence > b.confidence;
        });

    // 6.5 【额外去重】解决"大框套小框"的重叠冗余。
    //     YOLO 多尺度输出会对同一裂缝同时给出大框与小框, 此时标准
    //     IoU-NMS 算的是 交集/并集: 小框完全落在大框内时 IoU ≈
    //     小框面积/大框面积, 往往远低于 NMS 阈值(0.45), 于是两个框
    //     同时存活, 画出来就是"一个大框和一个小框高度重叠"。
    //     这里再用"覆盖比例"补压一轮: 若两个框中较小框的大部分
    //     (≥ kCoverRatio)落在较大框内, 判定为同一目标的冗余框,
    //     只保留置信度更高的那个 —— 要么一个大框框完, 要么多个小框,
    //     不再出现嵌套框。results 已按置信度降序, 所以后面的框置信度
    //     一定不高于前面的框, 被覆盖的直接删掉即可。
    constexpr float kCoverRatio = 0.7f;   ///< 较小框被较大框覆盖的比例阈值
    std::vector<CrackObject> dedup;
    dedup.reserve(results.size());
    for (size_t i = 0; i < results.size(); ++i)
    {
        bool redundant = false;
        for (size_t j = 0; j < i && !redundant; ++j)
        {
            // 前面的框 j 置信度 >= 当前框 i
            const int inter_area = std::max(0, (results[j].box & results[i].box).area());
            const int min_area = std::min(results[j].box.area(), results[i].box.area());
            if (min_area > 0 &&
                static_cast<float>(inter_area) / min_area >= kCoverRatio)
            {
                redundant = true;   // 与更高置信度框高度重叠 -> 冗余
            }
        }
        if (!redundant)
        {
            dedup.push_back(results[i]);
        }
    }
    results.swap(dedup);

    // 7. 合理性保护: 一张图正常只有几个裂缝。若一次检出的框数异常庞大
    //    (通常几百上千), 说明推理后端输出失真(如 GPU/OpenCL 后端问题),
    //    此时丢弃整帧结果, 避免把噪声当成裂缝。
    constexpr int kMaxReasonableCracks = 50;
    if (static_cast<int>(results.size()) > kMaxReasonableCracks)
    {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("YoloDetector"), steady_clock, 5000,
            "[YOLO] 检出 %zu 个框, 疑似推理后端输出异常, 已丢弃本帧",
            results.size());
        return {};
    }

    return results;
}

cv::Mat YoloDetector::decodeMask(
    const std::vector<float> & coeffs, const cv::Mat & proto,
    const cv::Mat & orig_img, int left, int top, float scale,
    const cv::Rect & box)
{
    if (coeffs.size() < 32 || proto.empty())
    {
        return cv::Mat();
    }

    // proto 布局: [1, 32, 160, 160] (NCHW) 或 [32, 160, 160] (CHW),
    // data 连续存放。统一取后三维作为 通道x高x宽。
    int c = 0, mh = 0, mw = 0;
    if (proto.dims == 4)
    {
        c = proto.size[1]; mh = proto.size[2]; mw = proto.size[3];
    }
    else if (proto.dims == 3)
    {
        c = proto.size[0]; mh = proto.size[1]; mw = proto.size[2];
    }
    else
    {
        return cv::Mat();
    }
    if (c != 32)
    {
        return cv::Mat();
    }

    // 1. 线性组合: mask_logits(y,x) = Σ_i coeff[i] * proto[i,y,x]
    //    结果为 160x160 的 float 掩膜 logits(对应 letterbox 640x640 输入)
    cv::Mat mask_logits(mh, mw, CV_32F);
    const float * proto_data = reinterpret_cast<const float *>(proto.data);
    float * out_data = reinterpret_cast<float *>(mask_logits.data);

    for (int y = 0; y < mh; ++y)
    {
        for (int x = 0; x < mw; ++x)
        {
            float sum = 0.0f;
            for (int i = 0; i < 32; ++i)
            {
                // proto[i] 的第 (y,x) 个元素: 偏移 = i*mh*mw + y*mw + x
                sum += coeffs[i] * proto_data[i * mh * mw + y * mw + x];
            }
            out_data[y * mw + x] = sum;
        }
    }

    // 2. 二值化: > 0 为裂缝
    cv::Mat mask_bin;
    cv::threshold(mask_logits, mask_bin, 0.0, 255, cv::THRESH_BINARY);

    // 3. 缩放到 640x640(letterbox 输入尺寸), 再裁剪掉灰边,
    //    得到与原图内容对应的掩膜区域, 最后缩放到原图尺寸。
    cv::Mat mask_u8;
    mask_bin.convertTo(mask_u8, CV_8UC1);

    const int target = input_size_;
    cv::Mat mask_640;
    cv::resize(mask_u8, mask_640, cv::Size(target, target),
               0, 0, cv::INTER_NEAREST);

    // 原图在 letterbox 里的实际区域: (left, top) 起, 宽高 = 原图尺寸*scale
    const int content_w = static_cast<int>(orig_img.cols * scale);
    const int content_h = static_cast<int>(orig_img.rows * scale);
    // 边界保护: 灰边裁剪后的区域不能超出 640x640
    const int crop_x = std::max(0, std::min(left, target - 1));
    const int crop_y = std::max(0, std::min(top, target - 1));
    const int crop_w = std::max(1, std::min(content_w, target - crop_x));
    const int crop_h = std::max(1, std::min(content_h, target - crop_y));

    cv::Mat mask_cropped = mask_640(cv::Rect(crop_x, crop_y, crop_w, crop_h));

    // 4. 缩放到原图尺寸(最近邻保持硬边缘)
    cv::Mat mask_full;
    cv::resize(
        mask_cropped, mask_full, cv::Size(orig_img.cols, orig_img.rows),
        0, 0, cv::INTER_NEAREST);

    // 5. 【关键】裁剪到检测框内(crop_mask 效果):
    //    prototype 掩膜可能超出框范围, 若不裁剪会出现"框外也有描边"。
    //    只保留 框与整图相交 的区域, 其余置 0。
    cv::Rect clipped = box & cv::Rect(0, 0, orig_img.cols, orig_img.rows);
    if (clipped.width <= 0 || clipped.height <= 0)
    {
        return cv::Mat::zeros(orig_img.size(), CV_8UC1);
    }
    // 全零掩膜, 只把"框内"的掩膜像素拷回去(框外自动为 0)
    cv::Mat mask_clipped = cv::Mat::zeros(mask_full.size(), CV_8UC1);
    mask_full(clipped).copyTo(mask_clipped(clipped));

    return mask_clipped;
}

}  // namespace culvert_core
