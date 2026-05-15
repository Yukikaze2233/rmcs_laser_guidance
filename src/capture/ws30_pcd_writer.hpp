#pragma once

#include <expected>
#include <filesystem>
#include <string>

#include "capture/ws30_types.hpp"

namespace rmcs_laser_guidance {

auto write_ws30_frame_as_pcd(const std::filesystem::path& path,
                             const Ws30PointFrame& frame)
    -> std::expected<void, std::string>;

} // namespace rmcs_laser_guidance
