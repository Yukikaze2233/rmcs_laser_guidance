#include "capture/ws30_capture_log.hpp"

#include <array>
namespace rmcs_laser_guidance {
namespace {

constexpr std::array<char, 8> kMagic{ 'W', 'S', '3', '0', 'L', 'O', 'G', '1' };

#pragma pack(push, 1)
struct RawLogEntryHeader {
    std::uint8_t stream = 0;
    std::uint64_t capture_unix_ns = 0;
    std::uint32_t payload_size = 0;
};
#pragma pack(pop)

} // namespace

Ws30RawLogWriter::Ws30RawLogWriter(std::ofstream stream) noexcept
    : stream_(std::move(stream)) {}

auto Ws30RawLogWriter::open(const std::filesystem::path& path)
    -> std::expected<Ws30RawLogWriter, std::string> {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path, std::ios::binary);
    if (!stream) return std::unexpected("failed to open WS30 raw log for writing");
    stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
    if (!stream) return std::unexpected("failed to write WS30 raw log header");
    return Ws30RawLogWriter(std::move(stream));
}

auto Ws30RawLogWriter::append(Ws30StreamKind stream,
                              std::uint64_t capture_unix_ns,
                              std::span<const std::byte> payload)
    -> std::expected<void, std::string> {
    RawLogEntryHeader header{
        .stream = static_cast<std::uint8_t>(stream),
        .capture_unix_ns = capture_unix_ns,
        .payload_size = static_cast<std::uint32_t>(payload.size()),
    };
    stream_.write(reinterpret_cast<const char*>(&header), sizeof(header));
    stream_.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    if (!stream_) return std::unexpected("failed to append WS30 raw log entry");
    return {};
}

Ws30RawLogReader::Ws30RawLogReader(std::ifstream stream) noexcept
    : stream_(std::move(stream)) {}

auto Ws30RawLogReader::open(const std::filesystem::path& path)
    -> std::expected<Ws30RawLogReader, std::string> {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) return std::unexpected("failed to open WS30 raw log for reading");

    std::array<char, kMagic.size()> magic{};
    stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (!stream) return std::unexpected("failed to read WS30 raw log header");
    if (magic != kMagic) return std::unexpected("invalid WS30 raw log header");
    return Ws30RawLogReader(std::move(stream));
}

auto Ws30RawLogReader::read_next()
    -> std::expected<std::optional<Ws30RawLogEntry>, std::string> {
    RawLogEntryHeader header{};
    stream_.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (stream_.eof()) return std::optional<Ws30RawLogEntry>{};
    if (!stream_) return std::unexpected("failed to read WS30 raw log entry header");

    std::vector<std::byte> payload(header.payload_size);
    stream_.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    if (!stream_) return std::unexpected("failed to read WS30 raw log payload");

    return Ws30RawLogEntry{
        .stream = static_cast<Ws30StreamKind>(header.stream),
        .capture_unix_ns = header.capture_unix_ns,
        .payload = std::move(payload),
    };
}

auto ws30_stream_kind_name(Ws30StreamKind kind) -> const char* {
    switch (kind) {
    case Ws30StreamKind::points: return "points";
    case Ws30StreamKind::imu: return "imu";
    case Ws30StreamKind::status: return "status";
    }
    return "unknown";
}

} // namespace rmcs_laser_guidance
