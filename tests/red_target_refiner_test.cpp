#include <cstdio>
#include <print>

#include <opencv2/imgproc.hpp>

#include "internal/red_target_refiner.hpp"
#include "test_utils.hpp"

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        rmcs_laser_guidance::RedTargetRefiner refiner;

        cv::Mat positive = cv::Mat::zeros(240, 320, CV_8UC3);
        cv::rectangle(positive, {120, 70}, {132, 150}, {0, 0, 255}, -1);
        cv::rectangle(positive, {176, 74}, {188, 154}, {0, 0, 255}, -1);
        cv::circle(positive, {40, 40}, 12, {255, 255, 255}, -1);

        const auto positive_result = refiner.refine(positive);
        require(positive_result.detected, "refiner should detect paired red bars");
        require_near(positive_result.center.x, 154.0F, 6.0F, "positive center.x");
        require_near(positive_result.center.y, 112.0F, 8.0F, "positive center.y");
        require(!positive_result.contour.empty(), "positive contour should not be empty");

        cv::Mat single_bar = cv::Mat::zeros(240, 320, CV_8UC3);
        cv::rectangle(single_bar, {140, 80}, {154, 165}, {0, 0, 255}, -1);
        const auto single_bar_result = refiner.refine(single_bar);
        require(!single_bar_result.detected, "single red bar should not pass paired-bar refinement");

        cv::Mat roi_image = cv::Mat::zeros(240, 320, CV_8UC3);
        cv::rectangle(roi_image, {40, 60}, {52, 140}, {0, 0, 255}, -1);
        cv::rectangle(roi_image, {96, 60}, {108, 140}, {0, 0, 255}, -1);
        const auto roi_result = refiner.refine(roi_image, {20, 40, 120, 140});
        require(roi_result.detected, "ROI refinement should detect paired red bars");
        require_near(roi_result.center.x, 74.0F, 6.0F, "roi center.x");
        require_near(roi_result.center.y, 100.0F, 8.0F, "roi center.y");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "red_target_refiner_test failed: {}", e.what());
        return 1;
    }
}
