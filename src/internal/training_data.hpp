#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace rmcs_laser_guidance {

struct VideoSessionMetadata {
    std::string session_id { };
    std::filesystem::path relative_video_path { "raw.avi" };
    std::filesystem::path device_path { };
    int width = 0;
    int height = 0;
    double framerate = 0.0;
    std::string fourcc { };
    std::int64_t capture_start_unix_ms = 0;
    std::int64_t duration_ms = 0;
    std::string lighting_tag { };
    std::string background_tag { };
    std::string distance_tag { };
    std::string target_color { };
    bool operator_note_present = false;
};

struct ExportedTrainingFrame {
    std::string image_name { };
    std::string source_session_id { };
    std::int64_t source_timestamp_ms = 0;
    std::string split { };
    double blur_score = 0.0;
    int width = 0;
    int height = 0;
    std::filesystem::path relative_image_path { };
};

auto format_session_id(std::chrono::system_clock::time_point capture_start) -> std::string;
auto write_video_session_metadata(
    const std::filesystem::path& path, const VideoSessionMetadata& metadata) -> void;
auto load_video_session_metadata(const std::filesystem::path& path) -> VideoSessionMetadata;

auto blur_score_for_frame(const cv::Mat& image) -> double;
auto export_training_frames(const std::filesystem::path& video_path, std::string_view session_id,
    const std::filesystem::path& dataset_root, std::string_view split,
    std::int64_t sample_interval_ms) -> std::vector<ExportedTrainingFrame>;
auto write_export_manifest(
    const std::filesystem::path& path, const std::vector<ExportedTrainingFrame>& frames) -> void;

} // namespace rmcs_laser_guidance
