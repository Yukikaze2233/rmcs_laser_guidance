#pragma once

#include <cstddef>
#include <expected>
#include <span>
#include <string>

#include "capture/ws30_types.hpp"

namespace rmcs_laser_guidance {

class Ws30PacketParser {
public:
    static auto parse(std::span<const std::byte> payload)
        -> std::expected<Ws30Packet, std::string>;
};

} // namespace rmcs_laser_guidance
