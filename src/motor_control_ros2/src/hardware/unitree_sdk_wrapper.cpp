/**
 * @file unitree_sdk_wrapper.cpp
 * @brief 宇树电机官方 SDK 动态包装器
 * 
 * 使用 dlopen 动态加载官方 SDK，避免编译时依赖
 */

#include <iostream>
#include <dlfcn.h>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>

namespace unitree {

// 电机类型
enum MotorType {
    A1 = 0,
    B1 = 1,
    GO_M8010_6 = 2
};

// 电机模式
enum MotorMode {
    BRAKE = 0,
    FOC = 1,
    CALIBRATE = 2
};

// 电机命令结构
struct MotorCmd {
    uint8_t mode;
    uint8_t id;
    float kp;
    float kd;
    float q;
    float dq;
    float tau;
};

// 电机反馈结构
struct MotorData {
    uint8_t mode;
    uint8_t id;
    float q;
    float dq;
    float tau;
    int temp;
    int merror;
};

class SerialPort {
public:
    SerialPort(const char* port_name) {
        // 打开串口
        fd_ = open(port_name, O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            throw std::runtime_error("Cannot open port");
        }
        
        // 配置串口
        struct termios tty;
        tcgetattr(fd_, &tty);
        
        // 设置波特率 4000000
        cfsetospeed(&tty, B4000000);
        cfsetispeed(&tty, B4000000);
        
        // 8N1
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag |= CREAD | CLOCAL;
        
        // 非规范模式
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ISIG;
        
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;  // 1秒超时
        
        tcsetattr(fd_, TCSANOW, &tty);
    }
    
    ~SerialPort() {
        if (fd_ >= 0) close(fd_);
    }
    
    // 宇树电机协议帧
    struct Frame {
        uint8_t head[2];    // FE EE
        uint8_t mode_id;    // bit0-3: ID, bit4-6: status
        uint8_t data[10];   // tor(2) + spd(2) + pos(4) + kp(2)
        uint8_t kd[2];      // kd
        uint8_t crc[2];     // CRC16
    };
    
    // 发送并接收
   
private:
    int fd_;
}; 
}
