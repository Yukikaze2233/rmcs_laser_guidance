#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>

namespace rmcs_laser_guidance {

struct Ws30UdpConfig {
    std::string bind_address = "0.0.0.0";
    std::uint16_t bind_port = 0;
    std::string remote_address{};
    std::uint16_t remote_port = 0;
    int receive_timeout_ms = 50;
};

class Ws30UdpSocket {
public:
    Ws30UdpSocket() = delete;
    Ws30UdpSocket(const Ws30UdpSocket&) = delete;
    auto operator=(const Ws30UdpSocket&) -> Ws30UdpSocket& = delete;
    Ws30UdpSocket(Ws30UdpSocket&& other) noexcept;
    auto operator=(Ws30UdpSocket&& other) noexcept -> Ws30UdpSocket&;
    ~Ws30UdpSocket() noexcept;

    [[nodiscard]] static auto open(const Ws30UdpConfig& config)
        -> std::expected<Ws30UdpSocket, std::string>;

    auto receive(std::span<std::byte> buffer)
        -> std::expected<std::size_t, std::string>;

    auto send_text(std::string_view message)
        -> std::expected<void, std::string>;

    auto close() noexcept -> void;

    [[nodiscard]] auto is_open() const noexcept -> bool { return fd_ >= 0; }

private:
    explicit Ws30UdpSocket(int fd, Ws30UdpConfig config) noexcept;

    int fd_ = -1;
    Ws30UdpConfig config_{};
};

} // namespace rmcs_laser_guidance
