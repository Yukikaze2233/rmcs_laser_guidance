#include "capture/ws30_frame_assembler.hpp"

namespace rmcs_laser_guidance {
namespace {

constexpr float kCoordinateScaleM = 0.002F;

auto make_point(const Ws30PointSample& sample, std::uint64_t timestamp_ms) -> Ws30Point {
    return Ws30Point{
        .x_m = static_cast<float>(sample.x_raw) * kCoordinateScaleM,
        .y_m = static_cast<float>(sample.y_raw) * kCoordinateScaleM,
        .z_m = static_cast<float>(sample.z_raw) * kCoordinateScaleM,
        .intensity = sample.intensity,
        .ring = static_cast<std::uint16_t>(sample.col / 4),
        .time_offset_s = static_cast<float>(timestamp_ms) * 0.001F,
        .row = sample.row,
        .col = sample.col,
    };
}

} // namespace

auto Ws30FrameAssembler::push(const Ws30PointsPacket& packet) -> std::optional<Ws30PointFrame> {
    if (packet.label == 0x00 || !active_) {
        reset();
        active_ = true;
        building_.timestamp_ms = packet.timestamp_ms;
    }

    append_packet(packet);

    if (packet.label != 0x02) return std::nullopt;

    active_ = false;
    auto completed = std::move(building_);
    building_ = {};
    return completed;
}

auto Ws30FrameAssembler::reset() -> void {
    active_ = false;
    building_ = {};
}

auto Ws30FrameAssembler::append_packet(const Ws30PointsPacket& packet) -> void {
    building_.points.reserve(building_.points.size() + packet.points.size());
    for (const auto& sample : packet.points) {
        building_.points.push_back(make_point(sample, packet.timestamp_ms));
    }
}

} // namespace rmcs_laser_guidance
