#include <filesystem>
#include <cstdio>
#include <print>

#include <opencv2/highgui.hpp>

#include "example_support.hpp"
#include "config.hpp"
#include "pipeline.hpp"
#include "internal/replay.hpp"

namespace {

auto resolve_replay_dir(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
    return rmcs_laser_guidance::examples::default_sample_replay_path();
}

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 2)
        return argv[2];
    return rmcs_laser_guidance::examples::default_config_path();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto replay_dir = resolve_replay_dir(argc, argv);
        const auto config_path = resolve_config_path(argc, argv);
        const auto config = rmcs_laser_guidance::load_config(config_path);
        const auto dataset = rmcs_laser_guidance::load_replay_dataset(replay_dir);
        rmcs_laser_guidance::Pipeline pipeline(config);

        for (const auto& frame_info : dataset.frames) {
            auto frame = rmcs_laser_guidance::load_replay_frame(dataset, frame_info);
            const auto observation = pipeline.process(frame);

            std::println(
                "frame={} detected={} brightness={}",
                frame_info.index,
                observation.detected,
                observation.brightness);

            if (config.debug.show_window) {
                cv::Mat display = frame.image.clone();
                pipeline.draw_debug_overlay(display, observation);
                cv::imshow("rmcs_laser_guidance_replay", display);
                if (rmcs_laser_guidance::examples::should_exit_from_key(cv::waitKey(1)))
                    break;
            }
        }

        return dataset.frames.empty() ? 1 : 0;
    } catch (const std::exception& e) {
        std::println(stderr, "example_replay_preview failed: {}", e.what());
        return 1;
    }
}
