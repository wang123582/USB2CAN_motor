#include "motor_control_ros2/serial_port.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <linux/serial.h>
#include <sys/ioctl.h>

namespace motor_control {

SerialPort::SerialPort(const std::string& port_name, int baudrate)
  : port_name_(port_name)
  , baudrate_(baudrate)
  , fd_(-1)
{}

SerialPort::~SerialPort() {
  close();
}

bool SerialPort::open() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[SerialPort] 无法打开串口: " << port_name_ << std::endl;
    return false;
  }
  
  if (!configurePort()) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  return true;
}

void SerialPort::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialPort::send(const uint8_t* data, size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) return -1;
  
  return ::write(fd_, data, len);
}

int SerialPort::receive(uint8_t* buffer, size_t max_len) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) return -1;
  
  return ::read(fd_, buffer, max_len);
}

bool SerialPort::configurePort() {
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[SerialPort] tcgetattr 失败" << std::endl;
    return false;
  }
  
  // 设置波特率
  // 注意：标准 termios 可能不支持 4.8Mbps，需要使用 BOTHER 或者 struct serial_struct
  // 这里暂时使用标准方式，如果失败可能需要更底层的设置
  speed_t speed;
  switch (baudrate_) {
    case 115200: speed = B115200; break;
    case 921600: speed = B921600; break;
    case 4000000: speed = B4000000; break; 
    default: 
      // 对于非标准波特率（如宇树的 4.8M），需要特殊处理
      // 这里先尝试 B4000000，实际宇树可能兼容或需要特殊 ioctl
      // 为简化，暂时设为 B921600 或 B4000000，具体视硬件支持
      speed = B921600; 
      std::cout << "[SerialPort] 警告: 使用默认波特率 921600，目标 " << baudrate_ << " 可能需要特殊驱动支持" << std::endl;
      break;
  }
  
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  // 8N1
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  
  // 无流控
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  
  // Raw 模式
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ISIG;
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  
  // 非阻塞读取设置
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;
  
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[SerialPort] tcsetattr 失败" << std::endl;
    return false;
  }
  
  // 尝试设置低延迟标志
  struct serial_struct serial;
  if (ioctl(fd_, TIOCGSERIAL, &serial) == 0) {
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &serial);
  }
  
  return true;
}

} // namespace motor_control
