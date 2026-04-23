#include "internal/red_target_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rmcs_laser_guidance {
namespace {

struct LightBarCandidate {
    cv::Point2f center;
    float long_side = 0.0F;
    float short_side = 0.0F;
    float angle_deg = 0.0F;
    float area = 0.0F;
    std::vector<cv::Point> contour;
};

auto normalize_roi(const cv::Mat& image, cv::Rect roi) -> cv::Rect {
    if (image.empty())
        return { };

    const cv::Rect full_image{0, 0, image.cols, image.rows};
    if (roi.width <= 0 || roi.height <= 0)
        return full_image;
    return roi & full_image;
}

auto to_bgr_image(const cv::Mat& image) -> cv::Mat {
    if (image.channels() == 3)
        return image;

    cv::Mat converted;
    if (image.channels() == 1) {
        cv::cvtColor(image, converted, cv::COLOR_GRAY2BGR);
        return converted;
    }

    if (image.channels() == 4) {
        cv::cvtColor(image, converted, cv::COLOR_BGRA2BGR);
        return converted;
    }

    throw std::runtime_error("unsupported image channel count for red target refinement");
}

auto red_mask_from_roi(const cv::Mat& roi_image) -> cv::Mat {
    cv::Mat hsv;
    cv::cvtColor(to_bgr_image(roi_image), hsv, cv::COLOR_BGR2HSV);

    cv::Mat lower_red;
    cv::Mat upper_red;
    cv::inRange(hsv, cv::Scalar(0, 120, 140), cv::Scalar(15, 255, 255), lower_red);
    cv::inRange(hsv, cv::Scalar(165, 120, 140), cv::Scalar(180, 255, 255), upper_red);

    cv::Mat mask;
    cv::bitwise_or(lower_red, upper_red, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    return mask;
}

auto long_axis_angle_deg(const cv::RotatedRect& rect) -> float {
    float angle = rect.angle;
    if (rect.size.width < rect.size.height)
        angle += 90.0F;
    return angle;
}

auto collect_light_bar_candidates(const cv::Mat& mask) -> std::vector<LightBarCandidate> {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBarCandidate> candidates;
    for (const auto& contour : contours) {
        const float area = static_cast<float>(cv::contourArea(contour));
        if (area < 12.0F)
            continue;

        const cv::RotatedRect rect = cv::minAreaRect(contour);
        const float long_side = std::max(rect.size.width, rect.size.height);
        const float short_side = std::min(rect.size.width, rect.size.height);
        if (short_side <= 0.0F)
            continue;

        const float aspect_ratio = long_side / short_side;
        if (aspect_ratio < 1.5F)
            continue;

        candidates.push_back(LightBarCandidate{
            .center = rect.center,
            .long_side = long_side,
            .short_side = short_side,
            .angle_deg = long_axis_angle_deg(rect),
            .area = area,
            .contour = contour,
        });
    }

    return candidates;
}

auto pair_score(const LightBarCandidate& lhs, const LightBarCandidate& rhs) -> float {
    const float size_similarity =
        std::min(lhs.long_side, rhs.long_side) / std::max(lhs.long_side, rhs.long_side);
    if (size_similarity < 0.5F)
        return 0.0F;

    const float angle_diff = std::abs(lhs.angle_deg - rhs.angle_deg);
    if (angle_diff > 25.0F)
        return 0.0F;

    const float center_distance = cv::norm(lhs.center - rhs.center);
    const float mean_length = 0.5F * (lhs.long_side + rhs.long_side);
    if (center_distance < mean_length * 0.5F || center_distance > mean_length * 6.0F)
        return 0.0F;

    const float vertical_offset = std::abs(lhs.center.y - rhs.center.y);
    if (vertical_offset > mean_length * 1.5F)
        return 0.0F;

    const float offset_penalty = 1.0F + (vertical_offset / std::max(mean_length, 1.0F));
    const float angle_penalty = 1.0F + (angle_diff / 10.0F);
    return ((lhs.area + rhs.area) * size_similarity) / (offset_penalty * angle_penalty);
}

auto merged_contour(const LightBarCandidate& lhs, const LightBarCandidate& rhs, const cv::Rect& roi)
    -> std::vector<cv::Point2f> {
    std::vector<cv::Point> points = lhs.contour;
    points.insert(points.end(), rhs.contour.begin(), rhs.contour.end());

    std::vector<cv::Point> hull;
    cv::convexHull(points, hull);

    std::vector<cv::Point2f> contour;
    contour.reserve(hull.size());
    for (const auto& point : hull) {
        contour.emplace_back(
            static_cast<float>(point.x + roi.x), static_cast<float>(point.y + roi.y));
    }
    return contour;
}

} // namespace

auto RedTargetRefiner::refine(const cv::Mat& image, const cv::Rect& roi) const
    -> RedTargetRefinement {
    const cv::Rect effective_roi = normalize_roi(image, roi);
    if (effective_roi.empty())
        return {.message = "refinement ROI is empty"};

    const cv::Mat roi_image = image(effective_roi).clone();
    const cv::Mat mask = red_mask_from_roi(roi_image);
    const auto candidates = collect_light_bar_candidates(mask);
    if (candidates.size() < 2U) {
        return {.message = "no valid red light-bar pair in ROI"};
    }

    float best_score = 0.0F;
    std::size_t best_lhs = 0;
    std::size_t best_rhs = 0;
    for (std::size_t lhs = 0; lhs < candidates.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < candidates.size(); ++rhs) {
            const float score = pair_score(candidates[lhs], candidates[rhs]);
            if (score > best_score) {
                best_score = score;
                best_lhs = lhs;
                best_rhs = rhs;
            }
        }
    }

    if (best_score <= std::numeric_limits<float>::epsilon()) {
        return {.message = "red components found, but no stable light-bar pair passed geometry checks"};
    }

    const auto& lhs = candidates[best_lhs];
    const auto& rhs = candidates[best_rhs];
    const cv::Point2f center = {
        effective_roi.x + 0.5F * (lhs.center.x + rhs.center.x),
        effective_roi.y + 0.5F * (lhs.center.y + rhs.center.y),
    };

    return {
        .detected = true,
        .center = center,
        .contour = merged_contour(lhs, rhs, effective_roi),
        .score = best_score,
        .message = "ok",
    };
}

} // namespace rmcs_laser_guidance
