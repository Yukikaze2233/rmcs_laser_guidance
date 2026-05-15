#include "capture/ws30_client.hpp"

#include <array>

#include "capture/ws30_packet_parser.hpp"

namespace rmcs_laser_guidance {

Ws30Client::Ws30Client(Ws30ClientConfig config)
    : config_(std::move(config)) {}

auto Ws30Client::open() -> std::expected<void, std::string> {
    auto points = open_socket(config_.points_port);
    if (!points) return std::unexpected(points.error());
    auto imu = open_socket(config_.imu_port);
    if (!imu) return std::unexpected(imu.error());
    auto status = open_socket(config_.status_port);
    if (!status) return std::unexpected(status.error());

    points_socket_.emplace(std::move(*points));
    imu_socket_.emplace(std::move(*imu));
    status_socket_.emplace(std::move(*status));
    assembler_.reset();
    return {};
}

auto Ws30Client::close() noexcept -> void {
    if (points_socket_) points_socket_->close();
    if (imu_socket_) imu_socket_->close();
    if (status_socket_) status_socket_->close();
    points_socket_.reset();
    imu_socket_.reset();
    status_socket_.reset();
    assembler_.reset();
}

auto Ws30Client::request_points_stream(bool enabled) -> std::expected<void, std::string> {
    if (!points_socket_) return std::unexpected("WS30 points socket is not open");
    return points_socket_->send_text(enabled ? "hello,points" : "stop,points");
}

auto Ws30Client::request_imu_stream(bool enabled) -> std::expected<void, std::string> {
    if (!imu_socket_) return std::unexpected("WS30 IMU socket is not open");
    return imu_socket_->send_text(enabled ? "hello,imu" : "stop,imu");
}

auto Ws30Client::request_serial_number() -> std::expected<void, std::string> {
    if (!status_socket_) return std::unexpected("WS30 status socket is not open");
    return status_socket_->send_text("sn");
}

auto Ws30Client::poll_points_frame() -> std::expected<std::optional<Ws30PointFrame>, std::string> {
    if (!points_socket_) return std::unexpected("WS30 points socket is not open");

    while (true) {
        auto packet = receive_packet(*points_socket_);
        if (!packet) return std::unexpected(packet.error());
        if (!packet->has_value()) return std::optional<Ws30PointFrame>{};
        if ((*packet)->kind != Ws30PacketKind::points) continue;
        const auto& points = std::get<Ws30PointsPacket>((*packet)->data);
        if (auto completed = assembler_.push(points); completed) return completed;
    }
}

auto Ws30Client::poll_imu_sample() -> std::expected<std::optional<Ws30ImuSample>, std::string> {
    if (!imu_socket_) return std::unexpected("WS30 IMU socket is not open");
    auto packet = receive_packet(*imu_socket_);
    if (!packet) return std::unexpected(packet.error());
    if (!packet->has_value()) return std::optional<Ws30ImuSample>{};
    if ((*packet)->kind != Ws30PacketKind::imu) return std::optional<Ws30ImuSample>{};

    const auto& imu = std::get<Ws30ImuPacket>((*packet)->data);
    return Ws30ImuSample{
        .timestamp_ms = imu.timestamp_ms,
        .gyro_x = imu.gyro_x,
        .gyro_y = imu.gyro_y,
        .gyro_z = imu.gyro_z,
        .acc_x = imu.acc_x,
        .acc_y = imu.acc_y,
        .acc_z = imu.acc_z,
    };
}

auto Ws30Client::poll_device_info() -> std::expected<std::optional<Ws30DeviceInfo>, std::string> {
    if (!status_socket_) return std::unexpected("WS30 status socket is not open");
    auto packet = receive_packet(*status_socket_);
    if (!packet) return std::unexpected(packet.error());
    if (!packet->has_value()) return std::optional<Ws30DeviceInfo>{};

    if ((*packet)->kind == Ws30PacketKind::serial_number) {
        const auto& sn = std::get<Ws30SerialNumberPacket>((*packet)->data);
        return Ws30DeviceInfo{
            .serial_number = sn.serial_number,
        };
    }
    if ((*packet)->kind == Ws30PacketKind::startup_status) {
        const auto& status = std::get<Ws30StartupStatusPacket>((*packet)->data);
        return Ws30DeviceInfo{
            .connected = status.connected,
        };
    }

    return std::optional<Ws30DeviceInfo>{};
}

auto Ws30Client::is_open() const noexcept -> bool {
    return points_socket_.has_value() && imu_socket_.has_value() && status_socket_.has_value();
}

auto Ws30Client::open_socket(std::uint16_t port) const -> std::expected<Ws30UdpSocket, std::string> {
    return Ws30UdpSocket::open(Ws30UdpConfig{
        .remote_address = config_.device_ip,
        .remote_port = port,
        .receive_timeout_ms = config_.receive_timeout_ms,
    });
}

auto Ws30Client::receive_packet(Ws30UdpSocket& socket) const
    -> std::expected<std::optional<Ws30Packet>, std::string> {
    std::array<std::byte, 4096> buffer{};
    const auto received = socket.receive(buffer);
    if (!received) {
        if (received.error().contains("Resource temporarily unavailable")) {
            return std::optional<Ws30Packet>{};
        }
        return std::unexpected(received.error());
    }

    auto parsed = Ws30PacketParser::parse(std::span<const std::byte>(buffer.data(), *received));
    if (!parsed) return std::unexpected(parsed.error());
    return std::optional<Ws30Packet>(std::move(parsed.value()));
}

} // namespace rmcs_laser_guidance
