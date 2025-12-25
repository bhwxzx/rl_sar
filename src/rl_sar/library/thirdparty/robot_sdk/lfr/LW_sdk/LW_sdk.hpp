#ifndef LW_SDK_HPP
#define LW_SDK_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#define MOTOR_COUNTS 10 

typedef struct {
    float pos_now;
    float vel_now;
    float tau_now;
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

#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];     // 0xA5, 0x5A
    float motor_action_set[MOTOR_COUNTS];
    float motor_kp_set[MOTOR_COUNTS];
    float motor_kd_set[MOTOR_COUNTS];
    uint8_t motors_disable;
    uint8_t tail;          // 0xFC
} MotorCmd_Packet_t;

typedef struct {
    uint8_t head[2];        // 0x55, 0xAA 
    float motor_pos_now[MOTOR_COUNTS];
    float motor_vel_now[MOTOR_COUNTS];
    float motor_tor_now[MOTOR_COUNTS];
    uint8_t tail;           // 0xED
} MotorFeedback_Packet_t;
#pragma pack(pop)

class LWSDK {
private:
    int serial_fd = -1;
    std::vector<uint8_t> rx_buffer;
    const size_t FEEDBACK_PACKET_SIZE = sizeof(MotorFeedback_Packet_t);

public:
    LWSDK() {
        rx_buffer.reserve(2048); // 预留空间减少重分配
    };
    
    ~LWSDK() {
        if (serial_fd >= 0) close(serial_fd);
    };

    void InitCmdData(LowCmd &cmd) {
        for (int i = 0; i < MOTOR_COUNTS; i++) {
            cmd.motorCmd[i].action_set = 0.0f;
            cmd.motorCmd[i].Kp = 0.0f;
            cmd.motorCmd[i].Kd = 0.0f;
        }
        cmd.motors_disable = false;
    }

    bool InitSerial(const char* port_name) {
        serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd < 0) {
            std::cerr << "[Error] Cannot open " << port_name << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd, &tty) != 0) return false;

        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);

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

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) return false;

        // 开启内核低延迟模式
        struct serial_struct ser_info;
        if (ioctl(serial_fd, TIOCGSERIAL, &ser_info) == 0) {
            ser_info.flags |= ASYNC_LOW_LATENCY;
            ioctl(serial_fd, TIOCSSERIAL, &ser_info);
        }

        tcflush(serial_fd, TCIOFLUSH);
        std::cout << "[Info] Serial initialized with Low Latency Mode: " << port_name << std::endl;
        return true;
    }

    void SendCmdData(LowCmd& cmd) {
        if (serial_fd < 0) return;

        static MotorCmd_Packet_t packet;
        packet.header[0] = 0xA5;
        packet.header[1] = 0x5A;
        packet.tail = 0xFC;

        for (int i=0; i < MOTOR_COUNTS; i++) {
            packet.motor_action_set[i] = cmd.motorCmd[i].action_set;
            packet.motor_kp_set[i] = cmd.motorCmd[i].Kp;
            packet.motor_kd_set[i] = cmd.motorCmd[i].Kd;
        }
        packet.motors_disable = cmd.motors_disable ? 0x01 : 0x00;

        write(serial_fd, &packet, sizeof(MotorCmd_Packet_t));
    }

    /**
     * @brief 优化后的接收函数
     * 逻辑：一次性读空串口缓冲区，在本地缓冲区寻找“最后一个”合法的包。
     */
    bool RecvFdData(LowState& state) {
        if (serial_fd < 0) return false;

        // 增大单次读取量
        uint8_t read_buf[1024]; 
        int n = read(serial_fd, read_buf, sizeof(read_buf));
        if (n > 0) {
            rx_buffer.insert(rx_buffer.end(), read_buf, read_buf + n);
        }

        if (rx_buffer.size() < FEEDBACK_PACKET_SIZE) return false;

        // 限制缓冲区，防止内存无限增长
        if (rx_buffer.size() > 4096) rx_buffer.clear();

        bool found_at_least_one = false;
        size_t last_valid_index = 0;
        bool has_valid_packet = false;

        // 在进入循环前增加一层判断
        if (rx_buffer.size() < FEEDBACK_PACKET_SIZE) return false;
        // --- 逆向或遍历寻找最新的一个完整包 ---
        // 从头往后扫描，不断更新“最新合法包”的位置
        for (size_t i = 0; i <= rx_buffer.size() - FEEDBACK_PACKET_SIZE; ) {
            if (rx_buffer[i] == 0x55 && rx_buffer[i+1] == 0xAA) {
                if (rx_buffer[i + FEEDBACK_PACKET_SIZE - 1] == 0xED) {
                    // 找到一个合法的包
                    last_valid_index = i;
                    has_valid_packet = true;
                    found_at_least_one = true;
                    // 继续往后找，看有没有更新的包
                    i += FEEDBACK_PACKET_SIZE;
                } else {
                    // 只有头没有尾，可能是噪声点，跳过这个头
                    i++;
                }
            } else {
                i++;
            }
        }

        if (has_valid_packet) {
            // 解析最新的包
            auto* p = reinterpret_cast<MotorFeedback_Packet_t*>(&rx_buffer[last_valid_index]);
            for (int i=0; i < MOTOR_COUNTS; i++) {
                state.motorState[i].pos_now = p->motor_pos_now[i];
                state.motorState[i].vel_now = p->motor_vel_now[i];
                state.motorState[i].tau_now = p->motor_tor_now[i];
            }

            // --- 一次性清空最后一个合法包之前的所有数据 ---
            // 这样可以确保 rx_buffer 里不会堆积旧包
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + last_valid_index + FEEDBACK_PACKET_SIZE);
        }

        return found_at_least_one;
    }
};

#endif