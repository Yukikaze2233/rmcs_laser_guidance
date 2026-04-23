#pragma once

#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs_laser_guidance {

struct RedTargetRefinement {
    bool detected = false;
    cv::Point2f center{-1.0F, -1.0F};
    std::vector<cv::Point2f> contour { };
    float score = 0.0F;
    std::string message { };
};

class RedTargetRefiner {
public:
    auto refine(const cv::Mat& image, const cv::Rect& roi = { }) const -> RedTargetRefinement;
};

} // namespace rmcs_laser_guidance
