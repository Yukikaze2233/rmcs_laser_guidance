#include <chrono>
#include <filesystem>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <print>
#include <string>
#include <string_view>

#include "ws30_lidar/capture_log.hpp"
#include "ws30_lidar/client.hpp"
#include "ws30_lidar/frame_assembler.hpp"
#include "ws30_lidar/packet_parser.hpp"
#include "ws30_lidar/pcd_writer.hpp"
#include "ws30_lidar/udp.hpp"

namespace {

using ws30_lidar::Client;
using ws30_lidar::ClientConfig;
using ws30_lidar::DeviceInfo;
using ws30_lidar::ImuSample;
using ws30_lidar::Packet;
using ws30_lidar::PacketKind;
using ws30_lidar::PacketParser;
using ws30_lidar::PointFrame;
using ws30_lidar::RawLogEntry;
using ws30_lidar::RawLogReader;
using ws30_lidar::RawLogWriter;
using ws30_lidar::StreamKind;
using ws30_lidar::UdpConfig;
using ws30_lidar::UdpSocket;
using ws30_lidar::write_frame_as_pcd;

struct Options {
    std::string device_ip = "192.168.137.200";
    std::uint16_t points_port = 1001;
    std::uint16_t imu_port = 1002;
    std::uint16_t status_port = 1003;
    int timeout_ms = 50;
    int iterations = 20;
    bool request_points = true;
    bool request_imu = true;
    std::filesystem::path record_raw_path{};
    std::filesystem::path replay_path{};
    std::filesystem::path write_pcd_dir{};
    int max_pcd_frames = -1;
};

auto usage() -> std::string_view {
    return "usage: tool_lidar_dump [--device-ip IPV4] [--points-port N] [--imu-port N] [--status-port N] [--timeout-ms N] [--iterations N] [--no-points] [--no-imu] [--record-raw FILE] [--replay FILE] [--write-pcd DIR] [--max-pcd-frames N]";
}

auto parse_u16(const char* value, const char* label) -> std::optional<std::uint16_t> {
    const auto parsed = std::strtol(value, nullptr, 10);
    if (parsed < 1 || parsed > 65535) {
        std::println(stderr, "tool_lidar_dump: invalid {} '{}'", label, value);
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(parsed);
}

auto parse_i32(const char* value, const char* label) -> std::optional<int> {
    const auto parsed = std::strtol(value, nullptr, 10);
    if (parsed < 0 || parsed > 1'000'000) {
        std::println(stderr, "tool_lidar_dump: invalid {} '{}'", label, value);
        return std::nullopt;
    }
    return static_cast<int>(parsed);
}

auto parse_options(int argc, char** argv) -> std::optional<Options> {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::println("{}", usage());
            return std::nullopt;
        }
        if (arg == "--no-points") {
            options.request_points = false;
            continue;
        }
        if (arg == "--no-imu") {
            options.request_imu = false;
            continue;
        }
        auto require_value = [&](const char* label) -> const char* {
            if (i + 1 >= argc) {
                std::println(stderr, "tool_lidar_dump: missing value for {}", label);
                return nullptr;
            }
            return argv[++i];
        };
        if (arg == "--device-ip") {
            if (const char* value = require_value("--device-ip")) options.device_ip = value;
            else return std::nullopt;
            continue;
        }
        if (arg == "--points-port") {
            const char* value = require_value("--points-port");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_u16(value, "points port");
            if (!parsed) return std::nullopt;
            options.points_port = *parsed;
            continue;
        }
        if (arg == "--imu-port") {
            const char* value = require_value("--imu-port");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_u16(value, "imu port");
            if (!parsed) return std::nullopt;
            options.imu_port = *parsed;
            continue;
        }
        if (arg == "--status-port") {
            const char* value = require_value("--status-port");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_u16(value, "status port");
            if (!parsed) return std::nullopt;
            options.status_port = *parsed;
            continue;
        }
        if (arg == "--timeout-ms") {
            const char* value = require_value("--timeout-ms");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_i32(value, "timeout-ms");
            if (!parsed) return std::nullopt;
            options.timeout_ms = *parsed;
            continue;
        }
        if (arg == "--record-raw") {
            if (const char* value = require_value("--record-raw")) options.record_raw_path = value;
            else return std::nullopt;
            continue;
        }
        if (arg == "--replay") {
            if (const char* value = require_value("--replay")) options.replay_path = value;
            else return std::nullopt;
            continue;
        }
        if (arg == "--write-pcd") {
            if (const char* value = require_value("--write-pcd")) options.write_pcd_dir = value;
            else return std::nullopt;
            continue;
        }
        if (arg == "--max-pcd-frames") {
            const char* value = require_value("--max-pcd-frames");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_i32(value, "max-pcd-frames");
            if (!parsed) return std::nullopt;
            options.max_pcd_frames = *parsed;
            continue;
        }
        if (arg == "--iterations") {
            const char* value = require_value("--iterations");
            if (value == nullptr) return std::nullopt;
            const auto parsed = parse_i32(value, "iterations");
            if (!parsed) return std::nullopt;
            options.iterations = *parsed;
            continue;
        }
        std::println(stderr, "tool_lidar_dump: unknown argument '{}'", arg);
        std::println(stderr, "{}", usage());
        return std::nullopt;
    }
    if (!options.record_raw_path.empty() && !options.replay_path.empty()) {
        std::println(stderr, "tool_lidar_dump: --record-raw and --replay cannot be used together");
        return std::nullopt;
    }
    return options;
}

auto print_points_frame(const PointFrame& frame) -> void {
    if (frame.points.empty()) {
        std::println("points frame ts={}ms count=0", frame.timestamp_ms);
        return;
    }
    const auto& first = frame.points.front();
    std::println("points frame ts={}ms count={} first[row={}, col={}, xyz=({:.3f},{:.3f},{:.3f})m ring={}]",
                 frame.timestamp_ms,
                 frame.points.size(),
                 first.row,
                 first.col,
                 first.x_m,
                 first.y_m,
                 first.z_m,
                 first.ring);
}

auto print_imu_sample(const ImuSample& imu) -> void {
    std::println("imu ts={}ms gyro=({:.3f},{:.3f},{:.3f}) acc=({:.3f},{:.3f},{:.3f})",
                 imu.timestamp_ms,
                 imu.gyro_x,
                 imu.gyro_y,
                 imu.gyro_z,
                 imu.acc_x,
                 imu.acc_y,
                 imu.acc_z);
}

auto to_imu_sample(const ws30_lidar::ImuPacket& packet) -> ImuSample {
    return ImuSample{
        .timestamp_ms = packet.timestamp_ms,
        .gyro_x = packet.gyro_x,
        .gyro_y = packet.gyro_y,
        .gyro_z = packet.gyro_z,
        .acc_x = packet.acc_x,
        .acc_y = packet.acc_y,
        .acc_z = packet.acc_z,
    };
}

auto print_device_info(const DeviceInfo& info) -> void {
    if (info.serial_number.has_value()) {
        std::println("status serial-number '{}'", *info.serial_number);
    }
    if (info.connected.has_value()) {
        std::println("status startup connected={}", *info.connected ? 1 : 0);
    }
}

auto log_command_error(const char* label, const std::expected<void, std::string>& result) -> void {
    if (!result) std::println(stderr, "tool_lidar_dump: failed to {}: {}", label, result.error());
}

auto unix_now_ns() -> std::uint64_t {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

auto make_pcd_path(const std::filesystem::path& dir,
                   std::uint64_t timestamp_ms,
                   int frame_index) -> std::filesystem::path {
    return dir / std::format("ws30_{:06}_{}ms.pcd", frame_index, timestamp_ms);
}

auto maybe_write_pcd(const Options& options,
                     const PointFrame& frame,
                     int& written_frames) -> void {
    if (options.write_pcd_dir.empty()) return;
    if (options.max_pcd_frames >= 0 && written_frames >= options.max_pcd_frames) return;
    const auto path = make_pcd_path(options.write_pcd_dir, frame.timestamp_ms, written_frames);
    if (auto result = write_frame_as_pcd(path, frame); !result) {
        std::println(stderr, "tool_lidar_dump: failed to write PCD '{}': {}", path.string(), result.error());
        return;
    }
    ++written_frames;
    std::println("tool_lidar_dump: wrote PCD {}", path.string());
}

auto handle_points_packet(const Packet& packet,
                          ws30_lidar::FrameAssembler& assembler,
                          const Options& options,
                          int& written_frames) -> void {
    const auto& points = std::get<ws30_lidar::PointsPacket>(packet.data);
    if (auto frame = assembler.push(points); frame) {
        print_points_frame(*frame);
        maybe_write_pcd(options, *frame, written_frames);
    }
}

auto handle_replay_entry(const RawLogEntry& entry,
                         ws30_lidar::FrameAssembler& assembler,
                         const Options& options,
                         int& written_frames) -> void {
    auto parsed = PacketParser::parse(entry.payload);
    if (!parsed) {
        std::println(stderr, "replay {} parse error: {}", ws30_lidar::stream_kind_name(entry.stream), parsed.error());
        return;
    }

    switch (parsed->kind) {
    case PacketKind::points:
        handle_points_packet(*parsed, assembler, options, written_frames);
        break;
    case PacketKind::imu:
        print_imu_sample(to_imu_sample(std::get<ws30_lidar::ImuPacket>(parsed->data)));
        break;
    case PacketKind::serial_number:
        print_device_info(DeviceInfo{.serial_number = std::get<ws30_lidar::SerialNumberPacket>(parsed->data).serial_number});
        break;
    case PacketKind::startup_status:
        print_device_info(DeviceInfo{.connected = std::get<ws30_lidar::StartupStatusPacket>(parsed->data).connected});
        break;
    case PacketKind::unknown:
        std::println("replay {} unknown packet", ws30_lidar::stream_kind_name(entry.stream));
        break;
    }
}

auto run_replay(const Options& options) -> int {
    auto reader = RawLogReader::open(options.replay_path);
    if (!reader) {
        std::println(stderr, "tool_lidar_dump: failed to open replay file: {}", reader.error());
        return 1;
    }
    ws30_lidar::FrameAssembler assembler;
    int processed = 0;
    int written_frames = 0;
    while (options.iterations <= 0 || processed < options.iterations) {
        auto entry = reader->read_next();
        if (!entry) {
            std::println(stderr, "tool_lidar_dump: replay read failed: {}", entry.error());
            return 1;
        }
        if (!entry->has_value()) break;
        handle_replay_entry(**entry, assembler, options, written_frames);
        ++processed;
    }
    return 0;
}

auto receive_and_record(UdpSocket& socket,
                        StreamKind stream,
                        RawLogWriter* writer,
                        std::array<std::byte, 4096>& buffer)
    -> std::expected<std::optional<Packet>, std::string> {
    const auto received = socket.receive(buffer);
    if (!received) {
        if (received.error().contains("Resource temporarily unavailable")) return std::optional<Packet>{};
        return std::unexpected(received.error());
    }
    if (writer != nullptr) {
        if (auto append = writer->append(stream, unix_now_ns(), std::span<const std::byte>(buffer.data(), *received)); !append) {
            return std::unexpected(append.error());
        }
    }
    auto parsed = PacketParser::parse(std::span<const std::byte>(buffer.data(), *received));
    if (!parsed) return std::unexpected(parsed.error());
    return std::optional<Packet>(std::move(parsed.value()));
}

auto run_live_with_recording(const Options& options) -> int {
    auto points = UdpSocket::open(UdpConfig{.remote_address = options.device_ip, .remote_port = options.points_port, .receive_timeout_ms = options.timeout_ms});
    if (!points) {
        std::println(stderr, "tool_lidar_dump: failed to open points socket: {}", points.error());
        return 1;
    }
    auto imu = UdpSocket::open(UdpConfig{.remote_address = options.device_ip, .remote_port = options.imu_port, .receive_timeout_ms = options.timeout_ms});
    if (!imu) {
        std::println(stderr, "tool_lidar_dump: failed to open imu socket: {}", imu.error());
        return 1;
    }
    auto status = UdpSocket::open(UdpConfig{.remote_address = options.device_ip, .remote_port = options.status_port, .receive_timeout_ms = options.timeout_ms});
    if (!status) {
        std::println(stderr, "tool_lidar_dump: failed to open status socket: {}", status.error());
        return 1;
    }
    auto writer = RawLogWriter::open(options.record_raw_path);
    if (!writer) {
        std::println(stderr, "tool_lidar_dump: failed to open raw log writer: {}", writer.error());
        return 1;
    }

    if (options.request_points) log_command_error("request points", points->send_text("hello,points"));
    if (options.request_imu) log_command_error("request imu", imu->send_text("hello,imu"));
    log_command_error("request serial number", status->send_text("sn"));

    ws30_lidar::FrameAssembler assembler;
    std::array<std::byte, 4096> points_buffer{};
    std::array<std::byte, 4096> imu_buffer{};
    std::array<std::byte, 4096> status_buffer{};
    int written_frames = 0;

    for (int i = 0; i < options.iterations; ++i) {
        if (options.request_points) {
            auto packet = receive_and_record(*points, StreamKind::points, &*writer, points_buffer);
            if (!packet) std::println(stderr, "points: {}", packet.error());
            else if (packet->has_value()) handle_points_packet(**packet, assembler, options, written_frames);
            else std::println(stderr, "points: timeout waiting for packet");
        }
        if (options.request_imu) {
            auto packet = receive_and_record(*imu, StreamKind::imu, &*writer, imu_buffer);
            if (!packet) std::println(stderr, "imu: {}", packet.error());
            else if (packet->has_value() && (*packet)->kind == PacketKind::imu)
                print_imu_sample(to_imu_sample(std::get<ws30_lidar::ImuPacket>((*packet)->data)));
            else std::println(stderr, "imu: timeout waiting for packet");
        }
        auto info = receive_and_record(*status, StreamKind::status, &*writer, status_buffer);
        if (!info) std::println(stderr, "status: {}", info.error());
        else if (info->has_value()) {
            if ((*info)->kind == PacketKind::serial_number)
                print_device_info(DeviceInfo{.serial_number = std::get<ws30_lidar::SerialNumberPacket>((*info)->data).serial_number});
            else if ((*info)->kind == PacketKind::startup_status)
                print_device_info(DeviceInfo{.connected = std::get<ws30_lidar::StartupStatusPacket>((*info)->data).connected});
        } else std::println(stderr, "status: timeout waiting for packet");
    }

    if (options.request_points) log_command_error("stop points stream", points->send_text("stop,points"));
    if (options.request_imu) log_command_error("stop imu stream", imu->send_text("stop,imu"));
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && (std::string_view(argv[1]) == "--help" || std::string_view(argv[1]) == "-h")) {
        std::println("{}", usage());
        return 0;
    }
    const auto options = parse_options(argc, argv);
    if (!options.has_value()) return argc > 1 ? 1 : 0;

    try {
        if (!options->replay_path.empty()) return run_replay(*options);
        if (!options->record_raw_path.empty()) return run_live_with_recording(*options);

        Client client(ClientConfig{
            .device_ip = options->device_ip,
            .points_port = options->points_port,
            .imu_port = options->imu_port,
            .status_port = options->status_port,
            .receive_timeout_ms = options->timeout_ms,
        });
        if (auto result = client.open(); !result) {
            std::println(stderr, "tool_lidar_dump: failed to open WS30 client: {}", result.error());
            return 1;
        }

        if (options->request_points) {
            if (auto result = client.request_points_stream(true); !result) {
                std::println(stderr, "tool_lidar_dump: failed to request points: {}", result.error());
            }
        }
        if (options->request_imu) {
            if (auto result = client.request_imu_stream(true); !result) {
                std::println(stderr, "tool_lidar_dump: failed to request imu: {}", result.error());
            }
        }
        if (auto result = client.request_serial_number(); !result) {
            std::println(stderr, "tool_lidar_dump: failed to request serial number: {}", result.error());
        }

        std::println("tool_lidar_dump: device={} ports(points={}, imu={}, status={}) timeout={}ms iterations={}",
                     options->device_ip,
                     options->points_port,
                     options->imu_port,
                     options->status_port,
                     options->timeout_ms,
                     options->iterations);
        int written_frames = 0;

        for (int i = 0; i < options->iterations; ++i) {
            if (options->request_points) {
                const auto frame = client.poll_points_frame();
                if (!frame) std::println(stderr, "points: {}", frame.error());
                else if (frame->has_value()) {
                    print_points_frame(**frame);
                    maybe_write_pcd(*options, **frame, written_frames);
                }
                else std::println(stderr, "points: timeout waiting for complete frame");
            }
            if (options->request_imu) {
                const auto imu = client.poll_imu_sample();
                if (!imu) std::println(stderr, "imu: {}", imu.error());
                else if (imu->has_value()) print_imu_sample(**imu);
                else std::println(stderr, "imu: timeout waiting for sample");
            }
            const auto info = client.poll_device_info();
            if (!info) std::println(stderr, "status: {}", info.error());
            else if (info->has_value()) print_device_info(**info);
            else std::println(stderr, "status: timeout waiting for packet");
        }

        if (options->request_points) log_command_error("stop points stream", client.request_points_stream(false));
        if (options->request_imu) log_command_error("stop imu stream", client.request_imu_stream(false));
        client.close();
        return 0;
    } catch (const std::exception& e) {
        std::println(stderr, "tool_lidar_dump failed: {}", e.what());
        return 1;
    }
}
