#ifndef LW_SDK_HPP
#define LW_SDK_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>

#include <unistd.h>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <sys/ioctl.h>
#include <linux/serial.h>

#define MOTOR_COUNTS 10 // 电机数量

typedef struct
{
    float pos_now;                           // current angle (unit: radian)
    float vel_now;                          // current velocity (unit: radian/second)
    float tau_now;                      // current estimated output torque (unit: N.m)

} MotorState;                          // motor feedback

typedef struct
{
    float action_set;                  // 期望的目标动作， 对于关节电机是目标位置， 对于轮子电机是目标速度
    float Kp;                          // desired position stiffness (unit: N.m/rad )
    float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )

} MotorCmd;                            // motor control

// typedef struct
// {
//     float quaternion[4];               // quaternion, normalized, (w,x,y,z)
//     float gyroscope[3];                // angular velocity （unit: rad/s)
//     float accelerometer[3];            // m/(s2)
// } IMU;      

typedef struct
{
    // IMU imu;
    MotorState motorState[MOTOR_COUNTS];
} LowState;                            // low level feedback

typedef struct
{
    MotorCmd motorCmd[MOTOR_COUNTS];
    bool motors_disable;
} LowCmd;  

#pragma pack(push, 1) // 强制1字节对齐

// 发送给 STM32 的命令包 (124 字节)
typedef struct {
    uint8_t header[2];     // 固定为 0xA5, 0x5A
    float motor_action_set[MOTOR_COUNTS]; // 目标位置/速度
    float motor_kp_set[MOTOR_COUNTS];     // Kp
    float motor_kd_set[MOTOR_COUNTS];     // Kd
    uint8_t motors_disable;  // 0x00: Enable, 0x01: Disable
    uint8_t tail;          // 固定为 0xFC
} MotorCmd_Packet_t;

// 从 STM32 接收的反馈包 (123 字节)
typedef struct {
    uint8_t head[2];        // 固定为 0x55, 0xAA 
    float motor_pos_now[MOTOR_COUNTS];
    float motor_vel_now[MOTOR_COUNTS];
    float motor_tor_now[MOTOR_COUNTS];
    uint8_t tail;           // 固定为 0xED
} MotorFeedback_Packet_t;

#pragma pack(pop)

class LWSDK
{
private:
    int serial_fd = -1;
    std::vector<uint8_t> rx_buffer; // 接收环形缓冲区(动态数组模拟)

public:
    LWSDK() {};
    ~LWSDK(){
        if (serial_fd >= 0) close(serial_fd);
    };

    void InitCmdData(LowCmd &cmd)
    {
        for (int i = 0; i < MOTOR_COUNTS; i++)
        {
            cmd.motorCmd[i].action_set = 0.0f;
            cmd.motorCmd[i].Kp = 0.0f;
            cmd.motorCmd[i].Kd = 0.0f;
        }
        cmd.motors_disable = false;
    }

     /**
     * @brief 初始化串口
     * @param port_name 设备路径，如 "/dev/ttyACM0"
     * @return true 成功, false 失败
     */
    bool InitSerial(const char* port_name)
    {
        // 打开串口: 读写 | 不作为控制终端 | 非阻塞
        serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd < 0) {
            std::cerr << "[Error] Cannot open " << port_name << std::endl;
            return false;
        }

        // 获取当前配置
        struct termios tty;
        if (tcgetattr(serial_fd, &tty) != 0) {
            std::cerr << "[Error] tcgetattr failed" << std::endl;
            return false;
        }

        // 设置波特率 921600
        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);

        // 设置 8N1
        tty.c_cflag &= ~PARENB;        // 无校验
        tty.c_cflag &= ~CSTOPB;        // 1停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8数据位
        tty.c_cflag &= ~CRTSCTS;       // 无硬件流控
        tty.c_cflag |= CREAD | CLOCAL; // 打开接收

        // 设置 Raw 模式 (关键：禁止所有特殊字符处理)
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        // 设置非阻塞读取
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            std::cerr << "[Error] tcsetattr failed" << std::endl;
            return false;
        }

        // 清除 O_NDELAY 标志，使用 VMIN=0 实现非阻塞
        fcntl(serial_fd, F_SETFL, 0);
        tcflush(serial_fd, TCIOFLUSH);

        std::cout << "[Info] Serial initialized: " << port_name << std::endl;
        return true;
    }

    /**
     * @brief 发送控制命令
     * @param cmd Lowcmd命令结构体
     */
    void SendCmdData(LowCmd& cmd)
    {
        if (serial_fd < 0) return;

        MotorCmd_Packet_t motorcmd;
        // 填充固定的包头包尾，防止用户忘记
        motorcmd.header[0] = 0xA5;
        motorcmd.header[1] = 0x5A;
        motorcmd.tail = 0xFC;

        for (int i=0; i < MOTOR_COUNTS; i++)
        {
            motorcmd.motor_action_set[i] = cmd.motorCmd[i].action_set;
            motorcmd.motor_kp_set[i] = cmd.motorCmd[i].Kp;
            motorcmd.motor_kd_set[i] = cmd.motorCmd[i].Kd;
        }
        if (cmd.motors_disable == false)
        {
            motorcmd.motors_disable = 0x00;
        }
        else
        {
            motorcmd.motors_disable = 0x01;
        }

        int written = write(serial_fd, &motorcmd, sizeof(MotorCmd_Packet_t));
        if (written < 0) {
            // std::cerr << "Write error" << std::endl;
        }
    }

     /**
     * @brief 尝试接收数据，如果缓冲区有多个包，会一直读到最后一个包，消除延迟
     * @param state 输出参数，接收到的最新反馈数据
     * @return true 成功更新了至少一次数据, false 无新数据
     */
    bool RecvFdData(LowState& state)
    {
        if (serial_fd < 0) return false;

        // 从串口读取所有可用数据追加到缓冲区
        uint8_t buf[256];
        int n = read(serial_fd, buf, sizeof(buf));
        if (n > 0) {
            rx_buffer.insert(rx_buffer.end(), buf, buf + n);
        }

        // 安全保护：防止缓冲区过大
        if (rx_buffer.size() > 2048) {
            rx_buffer.clear();
            return false;
        }

        bool has_updated = false; // 标记是否更新过数据

        // 循环处理缓冲区，直到剩下的数据不足一个包
        while (rx_buffer.size() >= sizeof(MotorFeedback_Packet_t)) 
        {
            // 检查包头 0x55 0xAA
            if (rx_buffer[0] == 0x55 && rx_buffer[1] == 0xAA) 
            {
                // 检查包尾 0xED
                if (rx_buffer[sizeof(MotorFeedback_Packet_t) - 1] == 0xED) 
                {
                    // --- 校验通过 ---
                    MotorFeedback_Packet_t feedback;
                    // 拷贝数据
                    memcpy(&feedback, rx_buffer.data(), sizeof(MotorFeedback_Packet_t));

                    // 更新 state (如果循环多次，这里会不断覆盖旧数据，保留最新的)
                    for (int i=0; i < MOTOR_COUNTS; i++)
                    {
                        state.motorState[i].pos_now = feedback.motor_pos_now[i];
                        state.motorState[i].vel_now = feedback.motor_vel_now[i];
                        state.motorState[i].tau_now = feedback.motor_tor_now[i];
                    }
                    
                    // 移除已处理的数据
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(MotorFeedback_Packet_t));
                    
                    // 标记为已更新，继续循环检查是否有更新的包
                    has_updated = true; 
                } 
                else 
                {
                    // 包头对但包尾错（可能是假包头），丢弃第1个字节，继续找
                    rx_buffer.erase(rx_buffer.begin());
                }
            } 
            else 
            {
                // 包头不对，丢弃第1个字节
                rx_buffer.erase(rx_buffer.begin());
            }
        }

        // 只有当至少成功解析了一个包时，才返回 true
        return has_updated;
    }

};

#endif