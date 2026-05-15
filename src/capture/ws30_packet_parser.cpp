#include "capture/ws30_packet_parser.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace rmcs_laser_guidance {
namespace {

#pragma pack(push, 1)
struct RawPointsPacket {
    std::uint8_t data_type[2];
    std::uint64_t timestamp_ms;
    std::uint8_t label;
    std::uint16_t row[kWs30PointsPerPacket];
    std::uint16_t col[kWs30PointsPerPacket];
    std::uint8_t intensity[kWs30PointsPerPacket];
    std::int16_t point_x[kWs30PointsPerPacket];
    std::int16_t point_y[kWs30PointsPerPacket];
    std::int16_t point_z[kWs30PointsPerPacket];
};

struct RawImuPacket {
    std::uint8_t data_type[2];
    std::uint64_t timestamp_ms;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
};

struct RawStatusPacket {
    std::uint8_t data_type[2];
    char sn_data[64];
    bool is_connected;
};
#pragma pack(pop)

template <typename TRaw>
auto copy_raw(std::span<const std::byte> payload) -> TRaw {
    TRaw raw{};
    std::memcpy(&raw, payload.data(), sizeof(TRaw));
    return raw;
}

auto trim_c_string(const char* value, std::size_t size) -> std::string {
    const auto* end = std::find(value, value + size, '\0');
    return std::string(value, end);
}

} // namespace

auto ws30_packet_kind_name(Ws30PacketKind kind) -> const char* {
    switch (kind) {
    case Ws30PacketKind::points: return "points";
    case Ws30PacketKind::imu: return "imu";
    case Ws30PacketKind::serial_number: return "serial_number";
    case Ws30PacketKind::startup_status: return "startup_status";
    case Ws30PacketKind::unknown: return "unknown";
    }
    return "unknown";
}

auto Ws30PacketParser::parse(std::span<const std::byte> payload)
    -> std::expected<Ws30Packet, std::string> {
    if (payload.size() < 2) {
        return std::unexpected("WS30 packet too short for header");
    }

    const auto header0 = static_cast<std::uint8_t>(payload[0]);
    const auto header1 = static_cast<std::uint8_t>(payload[1]);

    if (header0 == 0x5A && header1 == 0xA5) {
        if (payload.size() != sizeof(RawPointsPacket)) {
            return std::unexpected("WS30 points packet size mismatch");
        }
        const auto raw = copy_raw<RawPointsPacket>(payload);
        Ws30PointsPacket points;
        points.timestamp_ms = raw.timestamp_ms;
        points.label = raw.label;
        for (std::size_t i = 0; i < kWs30PointsPerPacket; ++i) {
            points.points[i] = Ws30PointSample{
                .row = raw.row[i],
                .col = raw.col[i],
                .intensity = raw.intensity[i],
                .x_raw = raw.point_x[i],
                .y_raw = raw.point_y[i],
                .z_raw = raw.point_z[i],
            };
        }
        return Ws30Packet{
            .kind = Ws30PacketKind::points,
            .data = std::move(points),
        };
    }

    if (header0 == 0x1A && header1 == 0xA1) {
        if (payload.size() != sizeof(RawImuPacket)) {
            return std::unexpected("WS30 IMU packet size mismatch");
        }
        const auto raw = copy_raw<RawImuPacket>(payload);
        return Ws30Packet{
            .kind = Ws30PacketKind::imu,
            .data = Ws30ImuPacket{
                .timestamp_ms = raw.timestamp_ms,
                .gyro_x = raw.gyro_x,
                .gyro_y = raw.gyro_y,
                .gyro_z = raw.gyro_z,
                .acc_x = raw.acc_x,
                .acc_y = raw.acc_y,
                .acc_z = raw.acc_z,
            },
        };
    }

    if (header0 == 0x2A && header1 == 0xA2) {
        if (payload.size() != sizeof(RawStatusPacket)) {
            return std::unexpected("WS30 SN packet size mismatch");
        }
        const auto raw = copy_raw<RawStatusPacket>(payload);
        return Ws30Packet{
            .kind = Ws30PacketKind::serial_number,
            .data = Ws30SerialNumberPacket{
                .serial_number = trim_c_string(raw.sn_data, sizeof(raw.sn_data)),
            },
        };
    }

    if (header0 == 0x3A && header1 == 0xA3) {
        if (payload.size() != sizeof(RawStatusPacket)) {
            return std::unexpected("WS30 startup status packet size mismatch");
        }
        const auto raw = copy_raw<RawStatusPacket>(payload);
        return Ws30Packet{
            .kind = Ws30PacketKind::startup_status,
            .data = Ws30StartupStatusPacket{
                .connected = raw.is_connected,
            },
        };
    }

    return Ws30Packet{
        .kind = Ws30PacketKind::unknown,
        .data = Ws30UnknownPacket{
            .header0 = header0,
            .header1 = header1,
        },
    };
}

} // namespace rmcs_laser_guidance
