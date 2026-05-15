#pragma once

#include <cstddef>
#include <expected>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "capture/ws30_types.hpp"

namespace rmcs_laser_guidance {

struct Ws30RawLogEntry {
    Ws30StreamKind stream = Ws30StreamKind::points;
    std::uint64_t capture_unix_ns = 0;
    std::vector<std::byte> payload{};
};

class Ws30RawLogWriter {
public:
    static auto open(const std::filesystem::path& path)
        -> std::expected<Ws30RawLogWriter, std::string>;

    auto append(Ws30StreamKind stream,
                std::uint64_t capture_unix_ns,
                std::span<const std::byte> payload)
        -> std::expected<void, std::string>;

private:
    explicit Ws30RawLogWriter(std::ofstream stream) noexcept;

    std::ofstream stream_;
};

class Ws30RawLogReader {
public:
    static auto open(const std::filesystem::path& path)
        -> std::expected<Ws30RawLogReader, std::string>;

    auto read_next()
        -> std::expected<std::optional<Ws30RawLogEntry>, std::string>;

private:
    explicit Ws30RawLogReader(std::ifstream stream) noexcept;

    std::ifstream stream_;
};

} // namespace rmcs_laser_guidance
