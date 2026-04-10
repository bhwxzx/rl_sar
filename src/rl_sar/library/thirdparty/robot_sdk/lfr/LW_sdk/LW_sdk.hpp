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
    uint8_t head[2];        // 0x55, 0xAA 
    float motor_pos_now[BOARD_MOTOR_COUNTS];
    float motor_vel_now[BOARD_MOTOR_COUNTS];
    float motor_tor_now[BOARD_MOTOR_COUNTS];
    uint8_t motor_state[BOARD_MOTOR_COUNTS];
    uint16_t crc16;
    uint8_t tail;           // 0xED
} MotorFeedback_Packet_t;
#pragma pack(pop)

class LWSDK {
private:
    int serial_fd_right = -1;
    int serial_fd_left  = -1;
    std::vector<uint8_t> rx_buffer_right;
    std::vector<uint8_t> rx_buffer_left;
    const size_t FEEDBACK_PACKET_SIZE = sizeof(MotorFeedback_Packet_t);

    uint16_t CalculateCRC16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc >>= 1;
                    crc ^= 0xA001; // Modbus 多项式
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

     // 内部打开单一串口的辅助函数
    bool OpenPort(int& fd, const char* port_name) {
        fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "[Error] Cannot open " << port_name << std::endl;
            return false;
        }
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) return false;

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

        if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;

        struct serial_struct ser_info;
        if (ioctl(fd, TIOCGSERIAL, &ser_info) == 0) {
            ser_info.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
        tcflush(fd, TCIOFLUSH);
        std::cout << "[Info] Serial init OK: " << port_name << std::endl;
        return true;
    }

    // 内部处理单一串口接收的辅助函数
    bool ProcessSingleRx(int fd, std::vector<uint8_t>& rx_buffer, LowState& state, bool is_left) {
        if (fd < 0) return false;

        uint8_t read_buf[1024]; 
        int n = read(fd, read_buf, sizeof(read_buf));
        if (n > 0) {
            rx_buffer.insert(rx_buffer.end(), read_buf, read_buf + n);
        }

        if (rx_buffer.size() < FEEDBACK_PACKET_SIZE) return false;
        if (rx_buffer.size() > 4096) rx_buffer.clear();

        bool has_valid_packet = false;
        size_t last_valid_index = 0;

        for (size_t i = 0; i <= rx_buffer.size() - FEEDBACK_PACKET_SIZE; ) {
            if (rx_buffer[i] == 0x55 && rx_buffer[i+1] == 0xAA) {
                MotorFeedback_Packet_t* p_test = reinterpret_cast<MotorFeedback_Packet_t*>(&rx_buffer[i]);
                if (p_test->tail == 0xED) {
                    uint16_t calc_crc = CalculateCRC16((uint8_t*)p_test, FEEDBACK_PACKET_SIZE - 3);
                    if (calc_crc == p_test->crc16) {
                        last_valid_index = i;
                        has_valid_packet = true;
                        i += FEEDBACK_PACKET_SIZE; 
                        continue;
                    }
                }
            }
            i++; 
        }

        if (has_valid_packet) {
            auto* p = reinterpret_cast<MotorFeedback_Packet_t*>(&rx_buffer[last_valid_index]);
            
            // ======= 组装映射 =======
            // 右腿板子对应总数组的偶数索引 {0,2,4,6,8}
            // 左腿板子对应总数组的奇数索引 {1,3,5,7,9}
            int map_idx[BOARD_MOTOR_COUNTS];
            if (is_left) {
                int l_idx[] = {1, 3, 5, 7, 9};
                std::memcpy(map_idx, l_idx, sizeof(map_idx));
            } else {
                int r_idx[] = {0, 2, 4, 6, 8};
                std::memcpy(map_idx, r_idx, sizeof(map_idx));
            }

            for (int i=0; i < BOARD_MOTOR_COUNTS; i++) {
                int target_idx = map_idx[i];
                state.motorState[target_idx].pos_now = p->motor_pos_now[i];
                state.motorState[target_idx].vel_now = p->motor_vel_now[i];
                state.motorState[target_idx].tau_now = p->motor_tor_now[i];
                state.motorState[target_idx].state_now = p->motor_state[i];
            }

            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + last_valid_index + FEEDBACK_PACKET_SIZE);
            return true;
        }
        return false;
    }

public:
    LWSDK() {
        rx_buffer_right.reserve(4096); 
        rx_buffer_left.reserve(4096); 
    };
    
    ~LWSDK() {
        if (serial_fd_right >= 0) close(serial_fd_right);
        if (serial_fd_left >= 0) close(serial_fd_left);
    };

    void InitCmdData(LowCmd &cmd) {
        for (int i = 0; i < MOTOR_COUNTS; i++) {
            cmd.motorCmd[i].action_set = 0.0f;
            cmd.motorCmd[i].Kp = 0.0f;
            cmd.motorCmd[i].Kd = 0.0f;
        }
        cmd.motors_disable = false;
    }

    // 接口修改为传入两个串口名称
    bool InitSerial(const char* port_right, const char* port_left) { // 右腿串口， 左腿串口
        bool r_ok = OpenPort(serial_fd_right, port_right);
        bool l_ok = OpenPort(serial_fd_left, port_left);
        return r_ok && l_ok;
    }

    void SendCmdData(LowCmd& cmd) {
        MotorCmd_Packet_t pkt_r, pkt_l;
        
        // 组装共同的头尾
        pkt_r.header[0] = 0xA5; pkt_r.header[1] = 0x5A; pkt_r.tail = 0xFC;
        pkt_l.header[0] = 0xA5; pkt_l.header[1] = 0x5A; pkt_l.tail = 0xFC;
        
        pkt_r.motors_disable = cmd.motors_disable ? 0x01 : 0x00;
        pkt_l.motors_disable = cmd.motors_disable ? 0x01 : 0x00;

        // ======= 拆分映射 =======
        int right_idx[] = {0, 2, 4, 6, 8};
        int left_idx[]  = {1, 3, 5, 7, 9};

        for (int i=0; i < BOARD_MOTOR_COUNTS; i++) {
            // 右腿包提取
            pkt_r.motor_action_set[i] = cmd.motorCmd[right_idx[i]].action_set;
            pkt_r.motor_kp_set[i]     = cmd.motorCmd[right_idx[i]].Kp;
            pkt_r.motor_kd_set[i]     = cmd.motorCmd[right_idx[i]].Kd;
            // 左腿包提取
            pkt_l.motor_action_set[i] = cmd.motorCmd[left_idx[i]].action_set;
            pkt_l.motor_kp_set[i]     = cmd.motorCmd[left_idx[i]].Kp;
            pkt_l.motor_kd_set[i]     = cmd.motorCmd[left_idx[i]].Kd;
        }
        
        // 分别算 CRC 并发送
        if (serial_fd_right >= 0) {
            pkt_r.crc16 = CalculateCRC16((uint8_t*)&pkt_r, sizeof(MotorCmd_Packet_t) - 3);
            write(serial_fd_right, &pkt_r, sizeof(MotorCmd_Packet_t));
        }
        
        if (serial_fd_left >= 0) {
            pkt_l.crc16 = CalculateCRC16((uint8_t*)&pkt_l, sizeof(MotorCmd_Packet_t) - 3);
            write(serial_fd_left, &pkt_l, sizeof(MotorCmd_Packet_t));
        }
    }

    bool RecvFdData(LowState& state) {
        // 分别尝试读取并解析左右腿的最新状态
        bool r_updated = ProcessSingleRx(serial_fd_right, rx_buffer_right, state, false);
        bool l_updated = ProcessSingleRx(serial_fd_left, rx_buffer_left, state, true);
        
        // 只要有一边的状态更新了，就告诉上层我们拿到新数据了
        return r_updated || l_updated;
    }

    // 电机故障检测函数
    bool MotorsProtect(LowState& state){
        bool has_fault = false;
        
        for(int i = 0; i < MOTOR_COUNTS; i++) {
            uint8_t err_code = state.motorState[i].state_now;
            
            // 达妙电机状态：1 是 Enable，0 是 Disable，大于 1 全是硬件报错
            if (err_code > 1) {
                std::string err_msg = "Unknown Error (未知错误)";
                
                // 解析达妙协议中的故障码
                switch (err_code) {
                    case 0x08: err_msg = "Over Voltage (过压)"; break;
                    case 0x09: err_msg = "Under Voltage (欠压)"; break;
                    case 0x0A: err_msg = "Over Current (过流)"; break;
                    case 0x0B: err_msg = "MOS Over Temperature (MOS过温)"; break;
                    case 0x0C: err_msg = "Coil Over Temperature (线圈过温)"; break;
                    case 0x0D: err_msg = "Communication Loss (通讯丢失)"; break;
                    case 0x0E: err_msg = "Overload (过载)"; break;
                }

                // 打印报错信息 (十六进制和中文释义同时打印，方便排查)
                std::cerr << "\033[1;31m[HARDWARE FAULT] Motor " << i 
                          << " Fault! Code: 0x" << std::hex << (int)err_code << std::dec
                          << " - " << err_msg << "\033[0m" << std::endl;
                          
                has_fault = true;
            }
        }
        return has_fault;
    }
};

#endif