#include "ir_seepage_detector.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

struct TrackState {
    int ksize_odd_steps = (71 - 3) / 2;
    int diff_thresh = 15;
    int morph_close = 21;
    int min_area = 120;
};

int cameraIndexFromArgs(int argc, char** argv) {
    int index = 2;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--camera" || arg == "-c") && i + 1 < argc) {
            index = std::atoi(argv[++i]);
        } else if (arg.rfind("--camera=", 0) == 0) {
            index = std::atoi(arg.c_str() + 9);
        }
    }
    return index;
}

cv::Mat grayBgrFrom32f(const cv::Mat& src32f) {
    cv::Mat norm8u, bgr;
    cv::normalize(src32f, norm8u, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::cvtColor(norm8u, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

}  // namespace

int main(int argc, char** argv) {
    const int camera_index = cameraIndexFromArgs(argc, argv);

    cv::VideoCapture cap(camera_index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "无法打开 /dev/video" << camera_index
                  << "，请用 --camera N 切换索引。" << std::endl;
        return 1;
    }

    IrSeepageDetector detector;
    TrackState ui;

    const std::string win = "IR Seepage Detector";
    cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("ksize", win, &ui.ksize_odd_steps, 50);
    cv::createTrackbar("diff_thresh", win, &ui.diff_thresh, 40);
    cv::createTrackbar("close", win, &ui.morph_close, 41);
    cv::createTrackbar("min_area", win, &ui.min_area, 1500);

    std::cout << "按 q 退出。" << std::endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "取流失败。" << std::endl;
            break;
        }

        IrSeepageParams params;
        params.ksize = 3 + 2 * ui.ksize_odd_steps;
        params.diff_thresh = static_cast<float>(ui.diff_thresh);
        params.morph_close_ksize = ui.morph_close;
        params.min_area_px = static_cast<double>(ui.min_area);
        detector.setParams(params);

        const IrSeepageResult result = detector.detect(frame);
        cv::Mat view = result.background.empty()
                           ? cv::Mat()
                           : grayBgrFrom32f(result.background);
        if (view.empty()) {
            if (frame.channels() == 1) {
                cv::cvtColor(frame, view, cv::COLOR_GRAY2BGR);
            } else {
                view = frame.clone();
            }
        }
        IrSeepageDetector::drawOverlay(view, result);

        const double scale = (frame.cols < 400) ? 3.0 : 1.5;
        cv::Mat display;
        cv::resize(view, display, cv::Size(), scale, scale, cv::INTER_NEAREST);
        cv::imshow(win, display);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
