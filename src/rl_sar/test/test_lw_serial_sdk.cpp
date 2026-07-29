#include "LW_sdk.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
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

MotorFeedback_Packet_t makeFeedback(float base)
{
    MotorFeedback_Packet_t packet{};
    packet.head[0] = 0x55;
    packet.head[1] = 0xAA;
    packet.tail = 0xED;
    for (int index = 0; index < BOARD_MOTOR_COUNTS; ++index)
    {
        packet.motor_pos_now[index] = base + static_cast<float>(index);
        packet.motor_vel_now[index] = base + 10.0f + static_cast<float>(index);
        packet.motor_tor_now[index] = base + 20.0f + static_cast<float>(index);
        packet.motor_state[index] = 1;
    }
    packet.crc16 = LWCalculateCRC16(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet) - 3);
    return packet;
}

std::vector<uint8_t> bytesOf(const MotorFeedback_Packet_t& packet)
{
    const auto* begin = reinterpret_cast<const uint8_t*>(&packet);
    return std::vector<uint8_t>(begin, begin + sizeof(packet));
}

void testParserHandlesNoiseSplitsAndMultiplePackets()
{
    LWFeedbackStreamParser parser;
    const auto first = makeFeedback(1.0f);
    const auto first_bytes = bytesOf(first);

    const std::array<uint8_t, 3> prefix = {0x19, 0x27, 0x55};
    auto result = parser.feed(prefix.data(), prefix.size());
    require(!result.updated(), "incomplete header produced a packet");
    require(parser.bufferedBytes() == 1, "split header byte was not retained");

    result = parser.feed(first_bytes.data() + 1, first_bytes.size() - 1);
    require(result.valid_packets == 1, "split packet was not recovered");
    require(result.latest_packet.has_value(), "split packet did not produce state");
    require(
        result.latest_packet->motor_pos_now[0] == first.motor_pos_now[0],
        "split packet payload changed");

    auto bad_crc = makeFeedback(10.0f);
    bad_crc.crc16 ^= 0x0101;
    auto bad_tail = makeFeedback(15.0f);
    bad_tail.tail = 0x00;
    const auto second = makeFeedback(20.0f);
    const auto latest = makeFeedback(30.0f);
    auto combined = bytesOf(bad_crc);
    const auto bad_tail_bytes = bytesOf(bad_tail);
    const auto second_bytes = bytesOf(second);
    const auto latest_bytes = bytesOf(latest);
    combined.insert(combined.end(), bad_tail_bytes.begin(), bad_tail_bytes.end());
    combined.insert(combined.end(), second_bytes.begin(), second_bytes.end());
    combined.insert(combined.end(), latest_bytes.begin(), latest_bytes.end());

    result = parser.feed(combined.data(), combined.size());
    require(result.crc_errors == 1, "bad CRC was not counted");
    require(result.format_errors == 1, "bad tail was not counted");
    require(result.valid_packets == 2, "multiple valid packets were not parsed");
    require(
        result.latest_packet->motor_pos_now[0] == latest.motor_pos_now[0],
        "latest valid packet was not selected");
}

void testParserRecoversFromOversizedNoise()
{
    LWFeedbackStreamParser parser;
    std::vector<uint8_t> noise(8192, 0x42);
    const auto result = parser.feed(noise.data(), noise.size());
    require(result.discarded_bytes == noise.size(), "oversized noise was not discarded");
    require(parser.bufferedBytes() == 0, "oversized noise remained buffered");

    const auto packet = makeFeedback(42.0f);
    const auto packet_bytes = bytesOf(packet);
    const auto recovered = parser.feed(packet_bytes.data(), packet_bytes.size());
    require(recovered.valid_packets == 1, "parser did not recover after oversized noise");
}

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

void writeAll(int fd, const uint8_t* data, size_t size)
{
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (offset < size)
    {
        const ssize_t written = ::write(fd, data + offset, size - offset);
        if (written > 0)
        {
            offset += static_cast<size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR)
        {
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            require(std::chrono::steady_clock::now() < deadline, "PTY write timed out");
            std::this_thread::yield();
            continue;
        }
        throw std::runtime_error("PTY write failed");
    }
}

std::vector<uint8_t> readExact(int fd, size_t size)
{
    std::vector<uint8_t> result(size);
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (offset < size)
    {
        const ssize_t bytes = ::read(fd, result.data() + offset, size - offset);
        if (bytes > 0)
        {
            offset += static_cast<size_t>(bytes);
            continue;
        }
        if (bytes < 0 && errno == EINTR)
        {
            continue;
        }
        if ((bytes == 0)
            || (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)))
        {
            require(std::chrono::steady_clock::now() < deadline, "PTY read timed out");
            std::this_thread::yield();
            continue;
        }
        throw std::runtime_error("PTY read failed");
    }
    return result;
}

void testPtyIntegration()
{
    auto right_pty = createPty();
    auto left_pty = createPty();

    LWSDK sdk;
    sdk.SetWriteTimeout(20ms);
    const auto init = sdk.InitSerial(right_pty.second.c_str(), left_pty.second.c_str());
    require(init.bothInitialized(), "PTY serial ports did not initialize");

    const auto right_feedback = makeFeedback(100.0f);
    const auto left_feedback = makeFeedback(200.0f);
    const auto right_bytes = bytesOf(right_feedback);
    const auto left_bytes = bytesOf(left_feedback);
    writeAll(right_pty.first.get(), right_bytes.data(), right_bytes.size());
    writeAll(left_pty.first.get(), left_bytes.data(), left_bytes.size());

    LowState state{};
    bool right_updated = false;
    bool left_updated = false;
    const auto receive_deadline = std::chrono::steady_clock::now() + 100ms;
    while (!(right_updated && left_updated))
    {
        const auto update = sdk.RecvFdData(state);
        require(!update.readFailed(), "PTY feedback read failed");
        right_updated = right_updated || update.right.updated;
        left_updated = left_updated || update.left.updated;
        require(
            std::chrono::steady_clock::now() < receive_deadline,
            "PTY feedback was not parsed");
        std::this_thread::yield();
    }

    require(state.motorState[0].pos_now == 100.0f, "right feedback mapping is wrong");
    require(state.motorState[1].pos_now == 200.0f, "left feedback mapping is wrong");
    require(state.motorState[8].pos_now == 104.0f, "right wheel mapping is wrong");
    require(state.motorState[9].pos_now == 204.0f, "left wheel mapping is wrong");

    LowCmd command{};
    sdk.InitCmdData(command);
    command.motors_disable = true;
    const auto send = sdk.SendCmdData(command);
    require(send.complete(), "complete PTY command was reported as failed");

    const auto right_command = readExact(right_pty.first.get(), sizeof(MotorCmd_Packet_t));
    const auto left_command = readExact(left_pty.first.get(), sizeof(MotorCmd_Packet_t));
    MotorCmd_Packet_t right_packet{};
    MotorCmd_Packet_t left_packet{};
    std::memcpy(&right_packet, right_command.data(), sizeof(right_packet));
    std::memcpy(&left_packet, left_command.data(), sizeof(left_packet));
    require(right_packet.motors_disable == 1, "right disable flag was lost");
    require(left_packet.motors_disable == 1, "left disable flag was lost");
    require(
        right_packet.crc16
            == LWCalculateCRC16(right_command.data(), sizeof(right_packet) - 3),
        "right command CRC is wrong");
    require(
        left_packet.crc16
            == LWCalculateCRC16(left_command.data(), sizeof(left_packet) - 3),
        "left command CRC is wrong");
}

void testFailedConfigurationDoesNotLeaveUsableDescriptors()
{
    const auto count_open_fds = []()
    {
        DIR* directory = ::opendir("/proc/self/fd");
        require(directory != nullptr, "cannot inspect /proc/self/fd");
        size_t count = 0;
        while (::readdir(directory) != nullptr)
        {
            ++count;
        }
        ::closedir(directory);
        return count;
    };

    const size_t before = count_open_fds();
    {
        LWSDK sdk;
        for (int attempt = 0; attempt < 20; ++attempt)
        {
            const auto init = sdk.InitSerial("/dev/null", "/definitely/missing/lw-port");
            require(!init.right.initialized, "non-TTY right path was accepted");
            require(!init.left.initialized, "missing left path was accepted");
        }

        LowCmd command{};
        sdk.InitCmdData(command);
        const auto send = sdk.SendCmdData(command);
        require(send.right.error_number == EBADF, "failed right port remained usable");
        require(send.left.error_number == EBADF, "failed left port remained usable");
        require(count_open_fds() == before, "failed initialization leaked a descriptor");
    }
    require(count_open_fds() == before, "LWSDK destruction leaked a descriptor");
}

void setNonBlocking(int fd)
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    require(flags >= 0, "F_GETFL failed");
    require(::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0, "F_SETFL failed");
}

void testPartialWritesAndWouldBlockReachDeadline()
{
    int right_pair[2] = {-1, -1};
    int left_pair[2] = {-1, -1};
    require(::socketpair(AF_UNIX, SOCK_STREAM, 0, right_pair) == 0, "right socketpair failed");
    require(::socketpair(AF_UNIX, SOCK_STREAM, 0, left_pair) == 0, "left socketpair failed");
    FileDescriptor right_writer(right_pair[0]);
    FileDescriptor right_reader(right_pair[1]);
    FileDescriptor left_writer(left_pair[0]);
    FileDescriptor left_reader(left_pair[1]);
    setNonBlocking(right_writer.get());
    setNonBlocking(left_writer.get());

    int send_buffer = 1024;
    require(
        ::setsockopt(
            right_writer.get(), SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer)) == 0,
        "right SO_SNDBUF failed");
    require(
        ::setsockopt(
            left_writer.get(), SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer)) == 0,
        "left SO_SNDBUF failed");

    std::vector<uint8_t> payload(1024 * 1024, 0x5A);
    const auto result = lw_detail::WritePacketPair(
        right_writer.get(),
        payload.data(),
        payload.size(),
        left_writer.get(),
        payload.data(),
        payload.size(),
        3ms);

    require(!result.complete(), "blocked sockets were reported as complete");
    require(result.right.bytes_written > 0, "right partial write was not tracked");
    require(result.left.bytes_written > 0, "left partial write was not tracked");
    require(
        result.right.bytes_written < payload.size(),
        "right blocked socket unexpectedly accepted the entire payload");
    require(
        result.left.bytes_written < payload.size(),
        "left blocked socket unexpectedly accepted the entire payload");
    require(result.right.error_number == ETIMEDOUT, "right timeout was not reported");
    require(result.left.error_number == ETIMEDOUT, "left timeout was not reported");
}

void testOneFailedPortDoesNotBlockTheOther()
{
    int pair[2] = {-1, -1};
    require(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0, "socketpair failed");
    FileDescriptor writer(pair[0]);
    FileDescriptor reader(pair[1]);
    setNonBlocking(writer.get());

    const std::array<uint8_t, 4> payload = {1, 2, 3, 4};
    const auto result = lw_detail::WritePacketPair(
        -1,
        payload.data(),
        payload.size(),
        writer.get(),
        payload.data(),
        payload.size(),
        10ms);

    require(result.right.error_number == EBADF, "invalid right fd was not reported");
    require(result.left.complete, "invalid right fd blocked the left packet");
    const auto received = readExact(reader.get(), payload.size());
    require(
        std::equal(received.begin(), received.end(), payload.begin()),
        "healthy side payload changed when peer side failed");
}
} // namespace

int main()
{
    try
    {
        testParserHandlesNoiseSplitsAndMultiplePackets();
        testParserRecoversFromOversizedNoise();
        testPtyIntegration();
        testFailedConfigurationDoesNotLeaveUsableDescriptors();
        testPartialWritesAndWouldBlockReachDeadline();
        testOneFailedPortDoesNotBlockTheOther();
        std::cout << "LW serial SDK tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW serial SDK test failed: " << exception.what() << std::endl;
        return 1;
    }
}
