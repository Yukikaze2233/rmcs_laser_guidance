#include "io/ws30_udp.hpp"

#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace rmcs_laser_guidance {
namespace {

auto make_sockaddr(const std::string& address, std::uint16_t port)
    -> std::expected<sockaddr_in, std::string> {
    sockaddr_in sockaddr{};
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, address.c_str(), &sockaddr.sin_addr) != 1) {
        return std::unexpected("invalid IPv4 address: " + address);
    }
    return sockaddr;
}

auto errno_message(const char* context) -> std::string {
    return std::string(context) + ": " + std::strerror(errno);
}

} // namespace

Ws30UdpSocket::Ws30UdpSocket(int fd, Ws30UdpConfig config) noexcept
    : fd_(fd), config_(std::move(config)) {}

Ws30UdpSocket::Ws30UdpSocket(Ws30UdpSocket&& other) noexcept
    : fd_(other.fd_), config_(std::move(other.config_)) {
    other.fd_ = -1;
}

auto Ws30UdpSocket::operator=(Ws30UdpSocket&& other) noexcept -> Ws30UdpSocket& {
    if (this == &other) return *this;
    close();
    fd_ = other.fd_;
    config_ = std::move(other.config_);
    other.fd_ = -1;
    return *this;
}

Ws30UdpSocket::~Ws30UdpSocket() noexcept {
    close();
}

auto Ws30UdpSocket::open(const Ws30UdpConfig& config)
    -> std::expected<Ws30UdpSocket, std::string> {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return std::unexpected(errno_message("WS30 socket create failed"));

    const timeval timeout{
        .tv_sec = config.receive_timeout_ms / 1000,
        .tv_usec = (config.receive_timeout_ms % 1000) * 1000,
    };
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
        const auto message = errno_message("WS30 set receive timeout failed");
        ::close(fd);
        return std::unexpected(message);
    }

    if (config.bind_port != 0 || config.bind_address != "0.0.0.0") {
        auto bind_addr = make_sockaddr(config.bind_address, config.bind_port);
        if (!bind_addr) {
            ::close(fd);
            return std::unexpected(bind_addr.error());
        }
        if (bind(fd, reinterpret_cast<const sockaddr*>(&*bind_addr), sizeof(*bind_addr)) != 0) {
            const auto message = errno_message("WS30 bind failed");
            ::close(fd);
            return std::unexpected(message);
        }
    }

    return Ws30UdpSocket(fd, config);
}

auto Ws30UdpSocket::receive(std::span<std::byte> buffer)
    -> std::expected<std::size_t, std::string> {
    if (!is_open()) return std::unexpected("WS30 socket is not open");
    const auto received = recvfrom(fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
    if (received < 0) return std::unexpected(errno_message("WS30 receive failed"));
    return static_cast<std::size_t>(received);
}

auto Ws30UdpSocket::send_text(std::string_view message)
    -> std::expected<void, std::string> {
    if (!is_open()) return std::unexpected("WS30 socket is not open");
    if (config_.remote_address.empty() || config_.remote_port == 0) {
        return std::unexpected("WS30 remote endpoint is not configured");
    }
    auto remote = make_sockaddr(config_.remote_address, config_.remote_port);
    if (!remote) return std::unexpected(remote.error());
    if (sendto(fd_, message.data(), message.size(), 0,
               reinterpret_cast<const sockaddr*>(&*remote), sizeof(*remote)) < 0) {
        return std::unexpected(errno_message("WS30 send failed"));
    }
    return {};
}

auto Ws30UdpSocket::close() noexcept -> void {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

} // namespace rmcs_laser_guidance
