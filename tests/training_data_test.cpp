#include <cstdio>
#include <filesystem>
#include <print>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "internal/training_data.hpp"
#include "test_utils.hpp"

namespace {

auto make_frame(const cv::Scalar& background, const bool with_edges) -> cv::Mat {
    cv::Mat image(120, 160, CV_8UC3, background);
    if (with_edges) {
        cv::rectangle(image, {30, 20}, {70, 90}, {255, 255, 255}, -1);
        cv::line(image, {10, 100}, {150, 30}, {0, 0, 255}, 3);
    }
    return image;
}

auto write_test_video(const std::filesystem::path& path) -> void {
    cv::VideoWriter writer(path.string(),
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        10.0,
        cv::Size(160, 120));
    if (!writer.isOpened())
        throw std::runtime_error("failed to open test video writer");

    writer.write(make_frame({0, 0, 0}, true));
    writer.write(make_frame({30, 30, 30}, false));
    writer.write(make_frame({0, 0, 0}, true));
    writer.write(make_frame({50, 50, 50}, false));
    writer.write(make_frame({0, 0, 0}, true));
    writer.release();
}

} // namespace

int main() {
    try {
        using namespace rmcs_laser_guidance::tests;

        const auto temp_root = make_temp_dir("rmcs_laser_guidance_training_data");
        const auto metadata_path = temp_root / "session.yaml";

        const rmcs_laser_guidance::VideoSessionMetadata metadata{
            .session_id = "20260423T123000",
            .relative_video_path = "raw.avi",
            .device_path = "/dev/video2",
            .width = 160,
            .height = 120,
            .framerate = 10.0,
            .fourcc = "MJPG",
            .capture_start_unix_ms = 123456789,
            .duration_ms = 500,
            .lighting_tag = "lab",
            .background_tag = "plain",
            .distance_tag = "20m",
            .target_color = "red",
            .operator_note_present = false,
        };
        rmcs_laser_guidance::write_video_session_metadata(metadata_path, metadata);
        const auto loaded = rmcs_laser_guidance::load_video_session_metadata(metadata_path);
        require(loaded.session_id == metadata.session_id, "session metadata session_id mismatch");
        require(loaded.device_path == metadata.device_path, "session metadata device_path mismatch");
        require(loaded.width == metadata.width, "session metadata width mismatch");
        require(loaded.height == metadata.height, "session metadata height mismatch");
        require(loaded.fourcc == metadata.fourcc, "session metadata fourcc mismatch");
        require(loaded.target_color == metadata.target_color, "session metadata target_color mismatch");

        cv::Mat sharp = make_frame({0, 0, 0}, true);
        cv::Mat blurred;
        cv::GaussianBlur(sharp, blurred, {9, 9}, 0.0);
        require(
            rmcs_laser_guidance::blur_score_for_frame(sharp)
                > rmcs_laser_guidance::blur_score_for_frame(blurred),
            "sharp frame should have higher blur score than blurred frame");

        const auto video_path = temp_root / "raw.avi";
        write_test_video(video_path);
        const auto dataset_root = temp_root / "dataset";
        const auto exported = rmcs_laser_guidance::export_training_frames(
            video_path, metadata.session_id, dataset_root, "train", 200);
        require(exported.size() == 3, "expected 3 exported frames from 5-frame test video");
        require(exported.front().source_timestamp_ms == 0, "first exported timestamp mismatch");
        require(exported.at(1).source_timestamp_ms == 200, "second exported timestamp mismatch");
        require(exported.at(2).source_timestamp_ms == 400, "third exported timestamp mismatch");

        for (const auto& frame : exported) {
            require(std::filesystem::exists(dataset_root / frame.relative_image_path),
                "exported frame image should exist");
            require(frame.split == "train", "exported frame split mismatch");
        }

        const auto manifest_path = dataset_root / "manifests" / "train_export_manifest.csv";
        rmcs_laser_guidance::write_export_manifest(manifest_path, exported);
        const std::string manifest_text = read_text_file(manifest_path);
        require_contains(manifest_text, "source_session_id", "export manifest header");
        require_contains(manifest_text, metadata.session_id, "export manifest session id");

        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "training_data_test failed: {}", e.what());
        return 1;
    }
}
