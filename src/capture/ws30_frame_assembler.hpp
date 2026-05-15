#pragma once

#include <optional>

#include "capture/ws30_types.hpp"

namespace rmcs_laser_guidance {

class Ws30FrameAssembler {
public:
    auto push(const Ws30PointsPacket& packet) -> std::optional<Ws30PointFrame>;
    auto reset() -> void;

private:
    auto append_packet(const Ws30PointsPacket& packet) -> void;

    Ws30PointFrame building_{};
    bool active_ = false;
};

} // namespace rmcs_laser_guidance
