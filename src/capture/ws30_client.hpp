#pragma once

#include <expected>
#include <optional>
#include <string>

#include "capture/ws30_frame_assembler.hpp"
#include "io/ws30_udp.hpp"

namespace rmcs_laser_guidance {

struct Ws30ClientConfig {
    std::string device_ip = "192.168.137.200";
    std::uint16_t points_port = 1001;
    std::uint16_t imu_port = 1002;
    std::uint16_t status_port = 1003;
    int receive_timeout_ms = 50;
};

class Ws30Client {
public:
    explicit Ws30Client(Ws30ClientConfig config);

    auto open() -> std::expected<void, std::string>;
    auto close() noexcept -> void;

    auto request_points_stream(bool enabled) -> std::expected<void, std::string>;
    auto request_imu_stream(bool enabled) -> std::expected<void, std::string>;
    auto request_serial_number() -> std::expected<void, std::string>;

    auto poll_points_frame() -> std::expected<std::optional<Ws30PointFrame>, std::string>;
    auto poll_imu_sample() -> std::expected<std::optional<Ws30ImuSample>, std::string>;
    auto poll_device_info() -> std::expected<std::optional<Ws30DeviceInfo>, std::string>;

    [[nodiscard]] auto is_open() const noexcept -> bool;

private:
    auto open_socket(std::uint16_t port) const -> std::expected<Ws30UdpSocket, std::string>;
    auto receive_packet(Ws30UdpSocket& socket) const
        -> std::expected<std::optional<Ws30Packet>, std::string>;

    Ws30ClientConfig config_{};
    std::optional<Ws30UdpSocket> points_socket_{};
    std::optional<Ws30UdpSocket> imu_socket_{};
    std::optional<Ws30UdpSocket> status_socket_{};
    Ws30FrameAssembler assembler_{};
};

} // namespace rmcs_laser_guidance
