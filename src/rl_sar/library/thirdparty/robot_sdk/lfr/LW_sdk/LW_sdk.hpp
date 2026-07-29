#ifndef LW_SDK_HPP
#define LW_SDK_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// 上层逻辑总电机数保持为 10 不变
#define MOTOR_COUNTS 10
// 底层每个单片机实际控制 5 个电机
#define BOARD_MOTOR_COUNTS 5

typedef struct {
    float pos_now;
    float vel_now;
    float tau_now;
    uint8_t state_now; // 电机的状态 (1为正常，>1为报错)
} MotorState;

typedef struct {
    float action_set;
    float Kp;
    float Kd;
} MotorCmd;

typedef struct {
    MotorState motorState[MOTOR_COUNTS];
} LowState;

typedef struct {
    MotorCmd motorCmd[MOTOR_COUNTS];
    bool motors_disable;
} LowCmd;

// 物理层通信包结构 (只针对单腿 5 个电机)
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];     // 0xA5, 0x5A
    float motor_action_set[BOARD_MOTOR_COUNTS];
    float motor_kp_set[BOARD_MOTOR_COUNTS];
    float motor_kd_set[BOARD_MOTOR_COUNTS];
    uint8_t motors_disable;
    uint16_t crc16;
    uint8_t tail;          // 0xFC
} MotorCmd_Packet_t;

typedef struct {
    uint8_t head[2];       // 0x55, 0xAA
    float motor_pos_now[BOARD_MOTOR_COUNTS];
    float motor_vel_now[BOARD_MOTOR_COUNTS];
    float motor_tor_now[BOARD_MOTOR_COUNTS];
    uint8_t motor_state[BOARD_MOTOR_COUNTS];
    uint16_t crc16;
    uint8_t tail;          // 0xED
} MotorFeedback_Packet_t;
#pragma pack(pop)

inline uint16_t LWCalculateCRC16(const uint8_t* data, size_t len) noexcept
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

struct LWFeedbackParseResult
{
    size_t valid_packets = 0;
    size_t discarded_bytes = 0;
    size_t crc_errors = 0;
    size_t format_errors = 0;
    std::optional<MotorFeedback_Packet_t> latest_packet;

    bool updated() const noexcept
    {
        return latest_packet.has_value();
    }
};

class LWFeedbackStreamParser
{
public:
    LWFeedbackStreamParser()
    {
        buffer_.reserve(2 * sizeof(MotorFeedback_Packet_t));
    }

    LWFeedbackParseResult feed(const uint8_t* data, size_t size)
    {
        if (data != nullptr && size > 0)
        {
            buffer_.insert(buffer_.end(), data, data + size);
        }

        LWFeedbackParseResult result;
        constexpr size_t packet_size = sizeof(MotorFeedback_Packet_t);
        size_t cursor = 0;

        while (buffer_.size() - cursor >= 2)
        {
            size_t header = cursor;
            while (header + 1 < buffer_.size()
                   && !(buffer_[header] == 0x55 && buffer_[header + 1] == 0xAA))
            {
                ++header;
            }

            if (header + 1 >= buffer_.size())
            {
                result.discarded_bytes += header - cursor;
                cursor = header;
                break;
            }

            result.discarded_bytes += header - cursor;
            cursor = header;
            if (buffer_.size() - cursor < packet_size)
            {
                break;
            }

            MotorFeedback_Packet_t packet{};
            std::memcpy(&packet, buffer_.data() + cursor, packet_size);

            if (packet.tail != 0xED)
            {
                ++result.format_errors;
                ++result.discarded_bytes;
                ++cursor;
                continue;
            }

            const uint16_t crc =
                LWCalculateCRC16(reinterpret_cast<const uint8_t*>(&packet), packet_size - 3);
            if (crc != packet.crc16)
            {
                ++result.crc_errors;
                ++result.discarded_bytes;
                ++cursor;
                continue;
            }

            ++result.valid_packets;
            result.latest_packet = packet;
            cursor += packet_size;
        }

        if (buffer_.size() - cursor == 1 && buffer_[cursor] != 0x55)
        {
            ++result.discarded_bytes;
            ++cursor;
        }

        if (cursor > 0)
        {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(cursor));
        }
        return result;
    }

    size_t bufferedBytes() const noexcept
    {
        return buffer_.size();
    }

    void reset() noexcept
    {
        buffer_.clear();
    }

private:
    std::vector<uint8_t> buffer_;
};

struct LWPortInitStatus
{
    bool initialized = false;
    int error_number = 0;
    std::string failed_operation;

    std::string failureDescription() const
    {
        if (initialized)
        {
            return "initialized";
        }

        std::string result = failed_operation.empty() ? "unknown operation" : failed_operation;
        if (error_number != 0)
        {
            result += ": ";
            result += std::strerror(error_number);
        }
        return result;
    }
};

struct LWSerialInitStatus
{
    LWPortInitStatus right;
    LWPortInitStatus left;

    bool bothInitialized() const noexcept
    {
        return right.initialized && left.initialized;
    }

    std::string failureSummary() const
    {
        std::string result;
        const auto append = [&result](const std::string& side, const LWPortInitStatus& status)
        {
            if (status.initialized)
            {
                return;
            }
            if (!result.empty())
            {
                result += "; ";
            }
            result += side + " (" + status.failureDescription() + ")";
        };
        append("right", right);
        append("left", left);
        return result;
    }
};

struct LWPortFeedbackStatus
{
    bool updated = false;
    size_t bytes_read = 0;
    size_t valid_packets = 0;
    size_t discarded_bytes = 0;
    size_t crc_errors = 0;
    size_t format_errors = 0;
    int error_number = 0;

    bool readFailed() const noexcept
    {
        return error_number != 0;
    }
};

struct LWFeedbackUpdate
{
    LWPortFeedbackStatus right;
    LWPortFeedbackStatus left;

    bool anyUpdated() const noexcept
    {
        return right.updated || left.updated;
    }

    bool bothUpdated() const noexcept
    {
        return right.updated && left.updated;
    }

    bool readFailed() const noexcept
    {
        return right.readFailed() || left.readFailed();
    }

    bool hasParserErrors() const noexcept
    {
        return right.crc_errors > 0 || right.format_errors > 0
            || left.crc_errors > 0 || left.format_errors > 0;
    }

    std::string parserErrorSummary() const
    {
        return "right crc=" + std::to_string(right.crc_errors)
             + ", format=" + std::to_string(right.format_errors)
             + "; left crc=" + std::to_string(left.crc_errors)
             + ", format=" + std::to_string(left.format_errors);
    }

    std::string failureSummary() const
    {
        std::string result;
        const auto append = [&result](const std::string& side, const LWPortFeedbackStatus& status)
        {
            if (!status.readFailed())
            {
                return;
            }
            if (!result.empty())
            {
                result += "; ";
            }
            result += side + " read: " + std::strerror(status.error_number);
        };
        append("right", right);
        append("left", left);
        return result;
    }
};

struct LWPortWriteStatus
{
    bool complete = false;
    size_t bytes_written = 0;
    int error_number = 0;

    bool failed() const noexcept
    {
        return !complete;
    }
};

struct LWSendResult
{
    LWPortWriteStatus right;
    LWPortWriteStatus left;

    bool complete() const noexcept
    {
        return right.complete && left.complete;
    }

    std::string failureSummary() const
    {
        std::string result;
        const auto append = [&result](const std::string& side, const LWPortWriteStatus& status)
        {
            if (status.complete)
            {
                return;
            }
            if (!result.empty())
            {
                result += "; ";
            }
            result += side + " wrote " + std::to_string(status.bytes_written)
                   + "/" + std::to_string(sizeof(MotorCmd_Packet_t)) + " bytes";
            if (status.error_number != 0)
            {
                result += ": ";
                result += std::strerror(status.error_number);
            }
        };
        append("right", right);
        append("left", left);
        return result;
    }
};

namespace lw_detail
{
struct WriteTarget
{
    int fd = -1;
    const uint8_t* data = nullptr;
    size_t size = 0;
    LWPortWriteStatus* status = nullptr;

    bool pending() const noexcept
    {
        return status != nullptr && !status->complete && status->error_number == 0;
    }
};

inline void setPendingError(
    std::array<WriteTarget, 2>& targets,
    int error_number) noexcept
{
    for (auto& target : targets)
    {
        if (target.pending())
        {
            target.status->error_number = error_number;
        }
    }
}

inline LWSendResult WritePacketPair(
    int right_fd,
    const uint8_t* right_data,
    size_t right_size,
    int left_fd,
    const uint8_t* left_data,
    size_t left_size,
    std::chrono::steady_clock::duration timeout) noexcept
{
    LWSendResult result;
    std::array<WriteTarget, 2> targets = {{
        {right_fd, right_data, right_size, &result.right},
        {left_fd, left_data, left_size, &result.left},
    }};

    for (auto& target : targets)
    {
        if (target.fd < 0)
        {
            target.status->error_number = EBADF;
        }
        else if (target.size == 0)
        {
            target.status->complete = true;
        }
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (targets[0].pending() || targets[1].pending())
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            setPendingError(targets, ETIMEDOUT);
            break;
        }

        for (auto& target : targets)
        {
            if (!target.pending())
            {
                continue;
            }

            const size_t remaining = target.size - target.status->bytes_written;
            const ssize_t written = ::write(
                target.fd,
                target.data + target.status->bytes_written,
                remaining);
            if (written > 0)
            {
                target.status->bytes_written += static_cast<size_t>(written);
                target.status->complete = target.status->bytes_written == target.size;
            }
            else if (written < 0)
            {
                const int write_error = errno;
                if (write_error != EINTR
                    && write_error != EAGAIN
                    && write_error != EWOULDBLOCK)
                {
                    target.status->error_number = write_error;
                }
            }
        }

        if (!(targets[0].pending() || targets[1].pending()))
        {
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            setPendingError(targets, ETIMEDOUT);
            break;
        }

        std::array<struct pollfd, 2> poll_fds{};
        std::array<size_t, 2> target_indices{};
        nfds_t poll_count = 0;
        for (size_t index = 0; index < targets.size(); ++index)
        {
            if (targets[index].pending())
            {
                poll_fds[poll_count].fd = targets[index].fd;
                poll_fds[poll_count].events = POLLOUT;
                target_indices[poll_count] = index;
                ++poll_count;
            }
        }

        const auto remaining_us =
            std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
        const int poll_timeout_ms =
            std::max(1, static_cast<int>((remaining_us + 999) / 1000));
        const int poll_result = ::poll(poll_fds.data(), poll_count, poll_timeout_ms);
        if (poll_result < 0)
        {
            const int poll_error = errno;
            if (poll_error != EINTR)
            {
                setPendingError(targets, poll_error);
                break;
            }
            continue;
        }
        if (poll_result == 0)
        {
            setPendingError(targets, ETIMEDOUT);
            break;
        }

        for (nfds_t index = 0; index < poll_count; ++index)
        {
            if (poll_fds[index].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                auto& target = targets[target_indices[index]];
                target.status->error_number =
                    (poll_fds[index].revents & POLLNVAL) ? EBADF : EIO;
            }
        }
    }

    return result;
}
} // namespace lw_detail

class LWSDK
{
public:
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;

    LWSDK() = default;

    ~LWSDK()
    {
        closePort(serial_fd_right_);
        closePort(serial_fd_left_);
    }

    LWSDK(const LWSDK&) = delete;
    LWSDK& operator=(const LWSDK&) = delete;

    void SetWriteTimeout(Duration timeout)
    {
        if (timeout <= Duration::zero())
        {
            throw std::invalid_argument("LW serial write timeout must be positive");
        }
        write_timeout_ = timeout;
    }

    void InitCmdData(LowCmd& cmd) const noexcept
    {
        for (int i = 0; i < MOTOR_COUNTS; ++i)
        {
            cmd.motorCmd[i].action_set = 0.0f;
            cmd.motorCmd[i].Kp = 0.0f;
            cmd.motorCmd[i].Kd = 0.0f;
        }
        cmd.motors_disable = false;
    }

    LWSerialInitStatus InitSerial(const char* port_right, const char* port_left)
    {
        right_parser_.reset();
        left_parser_.reset();

        LWSerialInitStatus status;
        status.right = OpenPort(serial_fd_right_, port_right);
        status.left = OpenPort(serial_fd_left_, port_left);
        return status;
    }

    LWSendResult SendCmdData(const LowCmd& cmd) noexcept
    {
        for (int index = 0; index < MOTOR_COUNTS; ++index)
        {
            const MotorCmd& motor = cmd.motorCmd[index];
            if (!std::isfinite(motor.action_set)
                || !std::isfinite(motor.Kp)
                || !std::isfinite(motor.Kd)
                || motor.Kp < 0.0f
                || motor.Kd < 0.0f)
            {
                LWSendResult invalid_result;
                invalid_result.right.error_number = EINVAL;
                invalid_result.left.error_number = EINVAL;
                return invalid_result;
            }
        }

        MotorCmd_Packet_t packet_right{};
        MotorCmd_Packet_t packet_left{};

        packet_right.header[0] = 0xA5;
        packet_right.header[1] = 0x5A;
        packet_right.tail = 0xFC;
        packet_left.header[0] = 0xA5;
        packet_left.header[1] = 0x5A;
        packet_left.tail = 0xFC;

        packet_right.motors_disable = cmd.motors_disable ? 0x01 : 0x00;
        packet_left.motors_disable = cmd.motors_disable ? 0x01 : 0x00;

        constexpr std::array<int, BOARD_MOTOR_COUNTS> right_indices = {0, 2, 4, 6, 8};
        constexpr std::array<int, BOARD_MOTOR_COUNTS> left_indices = {1, 3, 5, 7, 9};
        for (int i = 0; i < BOARD_MOTOR_COUNTS; ++i)
        {
            packet_right.motor_action_set[i] = cmd.motorCmd[right_indices[i]].action_set;
            packet_right.motor_kp_set[i] = cmd.motorCmd[right_indices[i]].Kp;
            packet_right.motor_kd_set[i] = cmd.motorCmd[right_indices[i]].Kd;

            packet_left.motor_action_set[i] = cmd.motorCmd[left_indices[i]].action_set;
            packet_left.motor_kp_set[i] = cmd.motorCmd[left_indices[i]].Kp;
            packet_left.motor_kd_set[i] = cmd.motorCmd[left_indices[i]].Kd;
        }

        packet_right.crc16 = LWCalculateCRC16(
            reinterpret_cast<const uint8_t*>(&packet_right),
            sizeof(MotorCmd_Packet_t) - 3);
        packet_left.crc16 = LWCalculateCRC16(
            reinterpret_cast<const uint8_t*>(&packet_left),
            sizeof(MotorCmd_Packet_t) - 3);

        return lw_detail::WritePacketPair(
            serial_fd_right_,
            reinterpret_cast<const uint8_t*>(&packet_right),
            sizeof(packet_right),
            serial_fd_left_,
            reinterpret_cast<const uint8_t*>(&packet_left),
            sizeof(packet_left),
            write_timeout_);
    }

    LWFeedbackUpdate RecvFdData(LowState& state) noexcept
    {
        LWFeedbackUpdate update;
        update.right =
            ProcessSingleRx(serial_fd_right_, right_parser_, state, false);
        update.left =
            ProcessSingleRx(serial_fd_left_, left_parser_, state, true);
        return update;
    }

    bool MotorsProtect(LowState& state)
    {
        bool has_fault = false;

        for (int i = 0; i < MOTOR_COUNTS; ++i)
        {
            const uint8_t err_code = state.motorState[i].state_now;

            // 达妙电机状态：1 是 Enable，0 是 Disable，大于 1 全是硬件报错
            if (err_code > 1)
            {
                std::string err_msg = "Unknown Error (未知错误)";

                switch (err_code)
                {
                    case 0x08: err_msg = "Over Voltage (过压)"; break;
                    case 0x09: err_msg = "Under Voltage (欠压)"; break;
                    case 0x0A: err_msg = "Over Current (过流)"; break;
                    case 0x0B: err_msg = "MOS Over Temperature (MOS过温)"; break;
                    case 0x0C: err_msg = "Coil Over Temperature (线圈过温)"; break;
                    case 0x0D: err_msg = "Communication Loss (通讯丢失)"; break;
                    case 0x0E: err_msg = "Overload (过载)"; break;
                }

                std::cerr << "\033[1;31m[HARDWARE FAULT] Motor " << i
                          << " Fault! Code: 0x" << std::hex << static_cast<int>(err_code)
                          << std::dec << " - " << err_msg << "\033[0m" << std::endl;
                has_fault = true;
            }
        }
        return has_fault;
    }

private:
    static constexpr size_t MAX_READ_BYTES_PER_CALL = 4096;

    int serial_fd_right_ = -1;
    int serial_fd_left_ = -1;
    LWFeedbackStreamParser right_parser_;
    LWFeedbackStreamParser left_parser_;
    Duration write_timeout_ = std::chrono::milliseconds(2);

    static void closePort(int& fd) noexcept
    {
        if (fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }

    static LWPortInitStatus OpenPort(int& fd, const char* port_name) noexcept
    {
        closePort(fd);
        LWPortInitStatus status;
        int candidate_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (candidate_fd < 0)
        {
            status.error_number = errno;
            status.failed_operation = "open";
            return status;
        }

        const auto fail = [&](const char* operation, int error_number)
        {
            status.error_number = error_number;
            status.failed_operation = operation;
            ::close(candidate_fd);
            candidate_fd = -1;
            return status;
        };

        struct termios tty{};
        if (::tcgetattr(candidate_fd, &tty) != 0)
        {
            return fail("tcgetattr", errno);
        }
        if (::cfsetospeed(&tty, B921600) != 0)
        {
            return fail("cfsetospeed", errno);
        }
        if (::cfsetispeed(&tty, B921600) != 0)
        {
            return fail("cfsetispeed", errno);
        }

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (::tcsetattr(candidate_fd, TCSANOW, &tty) != 0)
        {
            return fail("tcsetattr", errno);
        }

        struct serial_struct serial_info{};
        if (::ioctl(candidate_fd, TIOCGSERIAL, &serial_info) == 0)
        {
            serial_info.flags |= ASYNC_LOW_LATENCY;
            if (::ioctl(candidate_fd, TIOCSSERIAL, &serial_info) != 0)
            {
                std::cerr << "[Warning] Cannot enable ASYNC_LOW_LATENCY on "
                          << port_name << ": " << std::strerror(errno) << std::endl;
            }
        }

        if (::tcflush(candidate_fd, TCIOFLUSH) != 0)
        {
            return fail("tcflush", errno);
        }

        fd = candidate_fd;
        status.initialized = true;
        std::cout << "[Info] Serial init OK: " << port_name << std::endl;
        return status;
    }

    static void ApplyFeedbackPacket(
        const MotorFeedback_Packet_t& packet,
        LowState& state,
        bool is_left) noexcept
    {
        constexpr std::array<int, BOARD_MOTOR_COUNTS> right_indices = {0, 2, 4, 6, 8};
        constexpr std::array<int, BOARD_MOTOR_COUNTS> left_indices = {1, 3, 5, 7, 9};
        const auto& mapping = is_left ? left_indices : right_indices;

        for (int i = 0; i < BOARD_MOTOR_COUNTS; ++i)
        {
            const int target = mapping[i];
            state.motorState[target].pos_now = packet.motor_pos_now[i];
            state.motorState[target].vel_now = packet.motor_vel_now[i];
            state.motorState[target].tau_now = packet.motor_tor_now[i];
            state.motorState[target].state_now = packet.motor_state[i];
        }
    }

    static LWPortFeedbackStatus ProcessSingleRx(
        int fd,
        LWFeedbackStreamParser& parser,
        LowState& state,
        bool is_left) noexcept
    {
        LWPortFeedbackStatus status;
        if (fd < 0)
        {
            status.error_number = EBADF;
            return status;
        }

        std::optional<MotorFeedback_Packet_t> latest_packet;
        std::array<uint8_t, 1024> read_buffer{};
        while (status.bytes_read < MAX_READ_BYTES_PER_CALL)
        {
            const size_t capacity =
                std::min(read_buffer.size(), MAX_READ_BYTES_PER_CALL - status.bytes_read);
            const ssize_t bytes = ::read(fd, read_buffer.data(), capacity);
            if (bytes > 0)
            {
                status.bytes_read += static_cast<size_t>(bytes);
                const auto parse_result =
                    parser.feed(read_buffer.data(), static_cast<size_t>(bytes));
                status.valid_packets += parse_result.valid_packets;
                status.discarded_bytes += parse_result.discarded_bytes;
                status.crc_errors += parse_result.crc_errors;
                status.format_errors += parse_result.format_errors;
                if (parse_result.latest_packet)
                {
                    latest_packet = parse_result.latest_packet;
                }
                continue;
            }
            if (bytes == 0)
            {
                break;
            }

            const int read_error = errno;
            if (read_error == EINTR)
            {
                continue;
            }
            if (read_error == EAGAIN || read_error == EWOULDBLOCK)
            {
                break;
            }
            status.error_number = read_error;
            break;
        }

        if (latest_packet)
        {
            ApplyFeedbackPacket(*latest_packet, state, is_left);
            status.updated = true;
        }
        return status;
    }
};

#endif // LW_SDK_HPP
