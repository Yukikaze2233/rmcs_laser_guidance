#include <chrono>
#include <filesystem>
#include <cstdio>
#include <fstream>
#include <print>

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "config.hpp"
#include "example_support.hpp"
#include "internal/training_data.hpp"
#include "internal/v4l2_capture.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::examples::default_config_path();
}

auto resolve_output_root(int argc, char** argv) -> std::filesystem::path {
    if (argc > 2)
        return argv[2];
    return "/tmp/rmcs_laser_guidance_sessions";
}

auto resolve_duration_seconds(int argc, char** argv) -> double {
    if (argc > 3)
        return std::stod(argv[3]);
    return 30.0;
}

auto print_mode(const rmcs_laser_guidance::V4l2Config& requested,
                const rmcs_laser_guidance::V4l2NegotiatedFormat& actual) -> void {
    std::println(
        "requested device={} mode={}x{}@{} format={}",
        requested.device_path.string(),
        requested.width,
        requested.height,
        requested.framerate,
        rmcs_laser_guidance::examples::pixel_format_name(requested.pixel_format));
    std::println(
        "actual    device={} mode={}x{}@{} format={}",
        actual.device_path.string(),
        actual.width,
        actual.height,
        actual.framerate,
        actual.fourcc);
}

auto unix_time_milliseconds(const std::chrono::system_clock::time_point value) -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(value.time_since_epoch()).count();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = resolve_config_path(argc, argv);
        const auto output_root = resolve_output_root(argc, argv);
        const double duration_seconds = resolve_duration_seconds(argc, argv);
        if (duration_seconds <= 0.0) {
            std::println(stderr, "duration_seconds must be positive");
            return 1;
        }

        const auto config = rmcs_laser_guidance::load_config(config_path);
        rmcs_laser_guidance::V4l2Capture capture(config.v4l2);
        const auto open_result = capture.open();
        if (!open_result) {
            std::println(stderr, "Failed to open V4L2 capture: {}", open_result.error());
            return 1;
        }

        print_mode(config.v4l2, *open_result);

        const auto capture_start = std::chrono::system_clock::now();
        const std::string session_id = rmcs_laser_guidance::format_session_id(capture_start);
        const std::filesystem::path session_root = output_root / session_id;
        const std::filesystem::path video_path = session_root / "raw.avi";
        const std::filesystem::path metadata_path = session_root / "session.yaml";
        const std::filesystem::path notes_path = session_root / "notes.txt";

        std::filesystem::create_directories(session_root);

        const double fps = open_result->framerate > 0.0 ? open_result->framerate : config.v4l2.framerate;
        cv::VideoWriter writer(video_path.string(),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
            cv::Size(open_result->width, open_result->height));
        if (!writer.isOpened()) {
            std::println(stderr, "Failed to open session video for writing: {}", video_path.string());
            capture.close();
            return 1;
        }

        std::size_t recorded_frames = 0;
        const auto monotonic_start = std::chrono::steady_clock::now();
        const auto deadline = monotonic_start + std::chrono::duration<double>(duration_seconds);

        while (std::chrono::steady_clock::now() < deadline) {
            auto frame = capture.read_frame();
            if (!frame) {
                std::println(stderr, "Failed to read frame: {}", frame.error());
                continue;
            }

            writer.write(frame->image);
            ++recorded_frames;

            if (config.debug.show_window) {
                cv::imshow("rmcs_laser_guidance_record_session", frame->image);
                if (rmcs_laser_guidance::examples::should_exit_from_key(cv::waitKey(1)))
                    break;
            }
        }

        writer.release();
        capture.close();

        const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - monotonic_start)
                                     .count();
        rmcs_laser_guidance::write_video_session_metadata(metadata_path,
            rmcs_laser_guidance::VideoSessionMetadata{
                .session_id = session_id,
                .relative_video_path = "raw.avi",
                .device_path = open_result->device_path,
                .width = open_result->width,
                .height = open_result->height,
                .framerate = fps,
                .fourcc = open_result->fourcc,
                .capture_start_unix_ms = unix_time_milliseconds(capture_start),
                .duration_ms = duration_ms,
                .lighting_tag = "",
                .background_tag = "",
                .distance_tag = "",
                .target_color = "red",
                .operator_note_present = false,
            });

        std::ofstream notes(notes_path);
        if (!notes)
            throw std::runtime_error("failed to create session notes file");

        std::println("session_id={} recorded_frames={} session_root={}",
            session_id,
            recorded_frames,
            session_root.string());
        return recorded_frames > 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::println(stderr, "example_v4l2_record_session failed: {}", e.what());
        return 1;
    }
}
