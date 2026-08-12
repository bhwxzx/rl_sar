#include "lw_startup_disable.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class FileDescriptor
{
public:
    explicit FileDescriptor(int fd = -1) noexcept
        : fd_(fd)
    {
    }

    ~FileDescriptor()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    FileDescriptor(FileDescriptor&& other) noexcept
        : fd_(other.fd_)
    {
        other.fd_ = -1;
    }

    FileDescriptor& operator=(FileDescriptor&& other) noexcept
    {
        if (this != &other)
        {
            if (fd_ >= 0)
            {
                ::close(fd_);
            }
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    int get() const noexcept
    {
        return fd_;
    }

private:
    int fd_;
};

std::pair<FileDescriptor, std::string> createPty()
{
    const int master_fd = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    require(master_fd >= 0, "posix_openpt failed");
    FileDescriptor master(master_fd);
    require(::grantpt(master_fd) == 0, "grantpt failed");
    require(::unlockpt(master_fd) == 0, "unlockpt failed");
    const char* path = ::ptsname(master_fd);
    require(path != nullptr, "ptsname failed");
    return {std::move(master), std::string(path)};
}

LWStartupDisableConfig testConfig(
    const std::string& right_port,
    const std::string& left_port)
{
    LWStartupDisableConfig config;
    config.right_port = right_port;
    config.left_port = left_port;
    config.bootstrap_write_timeout = 20ms;
    config.initial_disable_attempts = 3;
    config.final_disable_attempts = 3;
    config.disable_interval = 2ms;
    return config;
}

std::vector<MotorCmd_Packet_t> readPackets(int fd)
{
    std::vector<std::uint8_t> bytes;
    auto quiet_deadline = std::chrono::steady_clock::now() + 20ms;
    while (std::chrono::steady_clock::now() < quiet_deadline)
    {
        struct pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = POLLIN;
        const int ready = ::poll(&descriptor, 1, 2);
        if (ready < 0 && errno == EINTR)
        {
            continue;
        }
        require(ready >= 0, "poll failed while reading PTY packets");
        if (ready == 0)
        {
            continue;
        }

        std::uint8_t buffer[4096];
        const ssize_t count = ::read(fd, buffer, sizeof(buffer));
        if (count > 0)
        {
            bytes.insert(bytes.end(), buffer, buffer + count);
            quiet_deadline = std::chrono::steady_clock::now() + 10ms;
            continue;
        }
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
        {
            continue;
        }
        if (count == 0 || (count < 0 && errno == EIO))
        {
            break;
        }
        require(false, "PTY read failed");
    }

    require(
        bytes.size() % sizeof(MotorCmd_Packet_t) == 0,
        "serial output ended with a partial command packet");
    std::vector<MotorCmd_Packet_t> packets(
        bytes.size() / sizeof(MotorCmd_Packet_t));
    for (std::size_t index = 0; index < packets.size(); ++index)
    {
        std::memcpy(
            &packets[index],
            bytes.data() + index * sizeof(MotorCmd_Packet_t),
            sizeof(MotorCmd_Packet_t));
    }
    return packets;
}

void requireDisablePackets(
    const std::vector<MotorCmd_Packet_t>& packets,
    std::size_t minimum_count,
    const std::string& context)
{
    require(
        packets.size() >= minimum_count,
        context + " produced too few disable packets");
    for (const auto& packet : packets)
    {
        require(packet.header[0] == 0xA5 && packet.header[1] == 0x5A,
                context + " produced an invalid packet header");
        require(packet.tail == 0xFC, context + " produced an invalid packet tail");
        require(packet.motors_disable == 1,
                context + " produced a motor-enable packet");
        require(
            packet.crc16
                == LWCalculateCRC16(
                    reinterpret_cast<const std::uint8_t*>(&packet),
                    sizeof(packet) - 3),
            context + " produced an invalid packet CRC");
    }
}

void testInitialDisablePrecedesFallibleWork()
{
    auto right = createPty();
    auto left = createPty();
    bool fallible_work_started = false;

    {
        LWStartupDisableGuard guard(testConfig(right.second, left.second));
        require(guard.initialDisableEstablished(),
                "startup guard returned without establishing disable output");
        require(guard.initialCompleteWrites() == 3,
                "startup guard did not complete the configured initial burst");
        require(guard.keepaliveStarted(),
                "startup guard did not start disable keepalive");
        fallible_work_started = true;
        const auto keepalive_deadline =
            std::chrono::steady_clock::now() + 100ms;
        while (guard.keepaliveCompleteWrites() == 0)
        {
            require(
                std::chrono::steady_clock::now() < keepalive_deadline,
                "startup disable keepalive did not write a packet");
            std::this_thread::yield();
        }
        guard.requireHealthy();
        guard.handOffToRuntime(20ms);
    }

    require(fallible_work_started, "fallible startup work was not reached");
    requireDisablePackets(readPackets(right.first.get()), 8, "right startup");
    requireDisablePackets(readPackets(left.first.get()), 8, "left startup");
}

void testPartialInitializationCannotReachFallibleWork()
{
    auto right = createPty();
    bool fallible_work_started = false;
    bool rejected = false;
    try
    {
        LWStartupDisableGuard guard(testConfig(
            right.second,
            "/definitely/missing/lw-startup-left"));
        fallible_work_started = true;
    }
    catch (const std::exception&)
    {
        rejected = true;
    }

    require(rejected, "partial serial initialization was accepted");
    require(!fallible_work_started,
            "fallible work started after partial serial initialization");
    requireDisablePackets(
        readPackets(right.first.get()),
        3,
        "partial initialization cleanup");
}

void testConstructorFailureRunsBoundedFinalDisable()
{
    auto right = createPty();
    auto left = createPty();
    bool injected_failure_seen = false;
    try
    {
        LWStartupDisableGuard guard(testConfig(right.second, left.second));
        guard.requireHealthy();
        injected_failure_seen = true;
        throw std::runtime_error("injected post-serial constructor failure");
    }
    catch (const std::runtime_error& exception)
    {
        require(
            std::string(exception.what())
                == "injected post-serial constructor failure",
            "unexpected constructor failure escaped the fixture");
    }

    require(injected_failure_seen, "constructor failure was not injected");
    requireDisablePackets(
        readPackets(right.first.get()),
        6,
        "right constructor cleanup");
    requireDisablePackets(
        readPackets(left.first.get()),
        6,
        "left constructor cleanup");
}

void testFinalDisableFollowsRuntimeCommand()
{
    auto right = createPty();
    auto left = createPty();
    {
        LWStartupDisableGuard guard(testConfig(right.second, left.second));
        guard.handOffToRuntime(20ms);

        LowCmd command{};
        guard.sdk().InitCmdData(command);
        command.motors_disable = false;
        LWSendResult result;
        const bool sent = guard.commandGate().sendIfOpen([&]()
        {
            result = guard.sdk().SendCmdData(command);
        });
        require(sent && result.complete(), "runtime command was not sent");
    }

    const auto right_packets = readPackets(right.first.get());
    const auto left_packets = readPackets(left.first.get());
    require(!right_packets.empty() && !left_packets.empty(),
            "runtime lifecycle produced no packets");
    require(right_packets.back().motors_disable == 1,
            "right lifecycle did not end with disable");
    require(left_packets.back().motors_disable == 1,
            "left lifecycle did not end with disable");

    bool right_runtime_seen = false;
    bool left_runtime_seen = false;
    for (const auto& packet : right_packets)
    {
        right_runtime_seen = right_runtime_seen || packet.motors_disable == 0;
    }
    for (const auto& packet : left_packets)
    {
        left_runtime_seen = left_runtime_seen || packet.motors_disable == 0;
    }
    require(right_runtime_seen && left_runtime_seen,
            "test did not observe the injected runtime command");
}
} // namespace

int main()
{
    try
    {
        testInitialDisablePrecedesFallibleWork();
        testPartialInitializationCannotReachFallibleWork();
        testConstructorFailureRunsBoundedFinalDisable();
        testFinalDisableFollowsRuntimeCommand();
        std::cout << "LW startup disable tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW startup disable test failed: "
                  << exception.what() << std::endl;
        return 1;
    }
}
