#include <filesystem>
#include <iostream>
#include <string>

#include <hikcamera/capturer.hpp>
#include <opencv2/highgui.hpp>

#include "rmcs_laser_guidance/config.hpp"
#include "rmcs_laser_guidance/pipeline.hpp"

namespace {

auto resolve_config_path(int argc, char** argv) -> std::filesystem::path {
    if (argc > 1)
        return argv[1];
#ifdef RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH
    return RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH;
#else
    return "configs/default.yaml";
#endif
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path config_path = resolve_config_path(argc, argv);
        const auto config = rmcs_laser_guidance::load_config(config_path);

        hikcamera::Camera camera;
        camera.configure(hikcamera::Config{
            .timeout_ms = config.camera.timeout_ms,
            .exposure_us = config.camera.exposure_us,
            .framerate = config.camera.framerate,
            .gain = config.camera.gain,
            .invert_image = config.camera.invert_image,
        });

        if (auto result = camera.connect(); !result) {
            std::cerr << "Failed to connect hikcamera: " << result.error() << '\n';
            return 1;
        }

        rmcs_laser_guidance::Pipeline pipeline(config);
        while (true) {
            auto image = camera.read_image_with_timestamp();
            if (!image) {
                std::cerr << "Failed to read image: " << image.error() << '\n';
                continue;
            }

            rmcs_laser_guidance::Frame frame{
                .image = image->mat,
                .timestamp = image->timestamp,
            };
            const auto observation = pipeline.process(frame);

            cv::Mat display = image->mat.clone();
            pipeline.draw_debug_overlay(display, observation);

            if (config.debug.show_window) {
                cv::imshow("rmcs_laser_guidance", display);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q')
                    break;
            }
        }

        if (camera.connected()) {
            auto disconnect_result = camera.disconnect();
            (void)disconnect_result;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "hikcamera_preview failed: " << e.what() << '\n';
        return 1;
    }
}
