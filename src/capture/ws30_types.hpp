#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <variant>

namespace rmcs_laser_guidance {

constexpr std::size_t kWs30PointsPerPacket = 120;

enum class Ws30PacketKind : std::uint8_t {
    points,
    imu,
    serial_number,
    startup_status,
    unknown,
};

enum class Ws30StreamKind : std::uint8_t {
    points = 0,
    imu = 1,
    status = 2,
};

struct Ws30PointSample {
    std::uint16_t row = 0;
    std::uint16_t col = 0;
    std::uint8_t intensity = 0;
    std::int16_t x_raw = 0;
    std::int16_t y_raw = 0;
    std::int16_t z_raw = 0;
};

struct Ws30Point {
    float x_m = 0.0F;
    float y_m = 0.0F;
    float z_m = 0.0F;
    std::uint8_t intensity = 0;
    std::uint16_t ring = 0;
    float time_offset_s = 0.0F;
    std::uint16_t row = 0;
    std::uint16_t col = 0;
};

struct Ws30PointsPacket {
    std::uint64_t timestamp_ms = 0;
    std::uint8_t label = 0;
    std::array<Ws30PointSample, kWs30PointsPerPacket> points{};
};

struct Ws30ImuPacket {
    std::uint64_t timestamp_ms = 0;
    float gyro_x = 0.0F;
    float gyro_y = 0.0F;
    float gyro_z = 0.0F;
    float acc_x = 0.0F;
    float acc_y = 0.0F;
    float acc_z = 0.0F;
};

struct Ws30SerialNumberPacket {
    std::string serial_number{};
};

struct Ws30StartupStatusPacket {
    bool connected = false;
};

struct Ws30UnknownPacket {
    std::uint8_t header0 = 0;
    std::uint8_t header1 = 0;
};

using Ws30PacketData = std::variant<Ws30PointsPacket,
                                    Ws30ImuPacket,
                                    Ws30SerialNumberPacket,
                                    Ws30StartupStatusPacket,
                                    Ws30UnknownPacket>;

struct Ws30Packet {
    Ws30PacketKind kind = Ws30PacketKind::unknown;
    Ws30PacketData data = Ws30UnknownPacket{};
};

struct Ws30PointFrame {
    std::uint64_t timestamp_ms = 0;
    std::vector<Ws30Point> points{};
};

struct Ws30ImuSample {
    std::uint64_t timestamp_ms = 0;
    float gyro_x = 0.0F;
    float gyro_y = 0.0F;
    float gyro_z = 0.0F;
    float acc_x = 0.0F;
    float acc_y = 0.0F;
    float acc_z = 0.0F;
};

struct Ws30DeviceInfo {
    std::optional<std::string> serial_number{};
    std::optional<bool> connected{};
};

auto ws30_packet_kind_name(Ws30PacketKind kind) -> const char*;
auto ws30_stream_kind_name(Ws30StreamKind kind) -> const char*;

} // namespace rmcs_laser_guidance
