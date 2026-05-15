#include <array>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <print>
#include <string>
#include <string_view>

#include "capture/ws30_client.hpp"

namespace {

using rmcs_laser_guidance::Ws30Client;
using rmcs_laser_guidance::Ws30ClientConfig;
using rmcs_laser_guidance::Ws30DeviceInfo;
using rmcs_laser_guidance::Ws30ImuSample;
using rmcs_laser_guidance::Ws30PointFrame;

struct Options {
    std::string device_ip = "192.168.137.200";
    std::uint16_t points_port = 1001;
    std::uint16_t imu_port = 1002;
    std::uint16_t status_port = 1003;
    int timeout_ms = 50;
    int iterations = 20;
    bool request_points = true;
    bool request_imu = true;
};

auto usage() -> std::string_view {
    return "usage: tool_lidar_dump [--device-ip IPV4] [--points-port N] [--imu-port N] [--status-port N] [--timeout-ms N] [--iterations N] [--no-points] [--no-imu]";
}

auto parse_u16(const char* value, const char* label) -> std::optional<std::uint16_t> {
    const auto parsed = std::strtol(value, nullptr, 10);
    if (parsed < 1 || parsed > 65535) {
        std::println(stderr, "tool_lidar_dump: invalid {} '{}'", label, value);
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(parsed);
}

auto parse_i32(const char* value, const char* label) -> std::optional<int> {
    const auto parsed = std::strtol(value, nullptr, 10);
    if (parsed < 0 || parsed > 1'000'000) {
        std::println(stderr, "tool_lidar_dump: invalid {} '{}'", label, value);
        return std::nullopt;
    }
    return static_cast<int>(parsed);
}

auto parse_options(int argc, char** argv) -> std::optional<Options> {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::println("{}", usage());
            return std::nullopt;
        }
        if (arg == "--no-points") {
            options.request_points = false;
            continue;
        }
        if (arg == "--no-imu") {
            options.request_imu = false;
            continue;
        }
        auto require_value = [&](const char* label) -> const char* {
            if (i + 1 >= argc) {
                std::println(stderr, "tool_lidar_dump: missing value for {}", label);
                return nullptr;
            }
            return argv[++i];
        };
        if (arg == "--device-ip") {
            if (const char* value = require_value("--device-ip")) options.device_ip = value;
            else return std::nullopt;
            continue;
        }
        if (arg == "--points-port") {
            const char* value = require_value("--points-port");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_u16(value, "points port");
            if (!parsed) return std::nullopt;
            options.points_port = *parsed;
            continue;
        }
        if (arg == "--imu-port") {
            const char* value = require_value("--imu-port");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_u16(value, "imu port");
            if (!parsed) return std::nullopt;
            options.imu_port = *parsed;
            continue;
        }
        if (arg == "--status-port") {
            const char* value = require_value("--status-port");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_u16(value, "status port");
            if (!parsed) return std::nullopt;
            options.status_port = *parsed;
            continue;
        }
        if (arg == "--timeout-ms") {
            const char* value = require_value("--timeout-ms");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_i32(value, "timeout-ms");
            if (!parsed) return std::nullopt;
            options.timeout_ms = *parsed;
            continue;
        }
        if (arg == "--iterations") {
            const char* value = require_value("--iterations");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_i32(value, "iterations");
            if (!parsed) return std::nullopt;
            options.iterations = *parsed;
            continue;
        }
        std::println(stderr, "tool_lidar_dump: unknown argument '{}'", arg);
        std::println(stderr, "{}", usage());
        return std::nullopt;
    }
    return options;
}

auto print_points_frame(const Ws30PointFrame& frame) -> void {
    if (frame.points.empty()) {
        std::println("points frame ts={}ms count=0", frame.timestamp_ms);
        return;
    }
    const auto& first = frame.points.front();
    std::println("points frame ts={}ms count={} first[row={}, col={}, xyz=({:.3f},{:.3f},{:.3f})m ring={}]",
                 frame.timestamp_ms,
                 frame.points.size(),
                 first.row,
                 first.col,
                 first.x_m,
                 first.y_m,
                 first.z_m,
                 first.ring);
}

auto print_imu_sample(const Ws30ImuSample& imu) -> void {
    std::println("imu ts={}ms gyro=({:.3f},{:.3f},{:.3f}) acc=({:.3f},{:.3f},{:.3f})",
                 imu.timestamp_ms,
                 imu.gyro_x,
                 imu.gyro_y,
                 imu.gyro_z,
                 imu.acc_x,
                 imu.acc_y,
                 imu.acc_z);
}

auto print_device_info(const Ws30DeviceInfo& info) -> void {
    if (info.serial_number.has_value()) {
        std::println("status serial-number '{}'", *info.serial_number);
    }
    if (info.connected.has_value()) {
        std::println("status startup connected={}", *info.connected ? 1 : 0);
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && (std::string_view(argv[1]) == "--help" || std::string_view(argv[1]) == "-h")) {
        std::println("{}", usage());
        return 0;
    }
    const auto options = parse_options(argc, argv);
    if (!options.has_value()) return argc > 1 ? 1 : 0;

    try {
        Ws30Client client(Ws30ClientConfig{
            .device_ip = options->device_ip,
            .points_port = options->points_port,
            .imu_port = options->imu_port,
            .status_port = options->status_port,
            .receive_timeout_ms = options->timeout_ms,
        });
        if (auto result = client.open(); !result) {
            std::println(stderr, "tool_lidar_dump: failed to open WS30 client: {}", result.error());
            return 1;
        }

        if (options->request_points) {
            if (auto result = client.request_points_stream(true); !result) {
                std::println(stderr, "tool_lidar_dump: failed to request points: {}", result.error());
            }
        }
        if (options->request_imu) {
            if (auto result = client.request_imu_stream(true); !result) {
                std::println(stderr, "tool_lidar_dump: failed to request imu: {}", result.error());
            }
        }
        if (auto result = client.request_serial_number(); !result) {
            std::println(stderr, "tool_lidar_dump: failed to request serial number: {}", result.error());
        }

        std::println("tool_lidar_dump: device={} ports(points={}, imu={}, status={}) timeout={}ms iterations={}",
                     options->device_ip,
                     options->points_port,
                     options->imu_port,
                     options->status_port,
                     options->timeout_ms,
                     options->iterations);

        for (int i = 0; i < options->iterations; ++i) {
            if (options->request_points) {
                const auto frame = client.poll_points_frame();
                if (!frame) std::println(stderr, "points: {}", frame.error());
                else if (frame->has_value()) print_points_frame(**frame);
                else std::println(stderr, "points: timeout waiting for complete frame");
            }
            if (options->request_imu) {
                const auto imu = client.poll_imu_sample();
                if (!imu) std::println(stderr, "imu: {}", imu.error());
                else if (imu->has_value()) print_imu_sample(**imu);
                else std::println(stderr, "imu: timeout waiting for sample");
            }
            const auto info = client.poll_device_info();
            if (!info) std::println(stderr, "status: {}", info.error());
            else if (info->has_value()) print_device_info(**info);
            else std::println(stderr, "status: timeout waiting for packet");
        }

        if (options->request_points) (void)client.request_points_stream(false);
        if (options->request_imu) (void)client.request_imu_stream(false);
        client.close();
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_lidar_dump failed: {}", e.what());
        return 1;
    }
}
