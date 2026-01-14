#include "motor_control_ros2/can_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace motor_control {

// ========== CANBus 实现 ==========

CANBus::CANBus(uint8_t bus_id, const std::string& port, int baudrate)
  : bus_id_(bus_id)
  , port_(port)
  , baudrate_(baudrate)
  , fd_(-1)
{
  memset(tx_buffer_, 0, sizeof(tx_buffer_));
  memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

CANBus::~CANBus() {
  close();
}

bool CANBus::open() {
  // 打开串口
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[CANBus] 无法打开串口: " << port_ << std::endl;
    return false;
  }
  
  // 配置串口
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[CANBus] tcgetattr 失败" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // 设置波特率
  speed_t speed = B921600;  // 默认 921600
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  // 8N1, 无流控
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ISIG;
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;
  
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[CANBus] tcsetattr 失败" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  std::cout << "[CANBus] 成功打开串口: " << port_ << std::endl;
  return true;
}

void CANBus::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CANBus::send(uint32_t can_id, const uint8_t* data, size_t len) {
  if (fd_ < 0 || len > 8) {
    return false;
  }
  
  // 构造30字节发送帧
  // 格式参考 Python 实现
  memset(tx_buffer_, 0, 30);
  
  // 帧头
  tx_buffer_[0] = 0xAA;
  tx_buffer_[1] = 0x55;
  
  // CAN ID (4 bytes, little endian)
  tx_buffer_[2] = can_id & 0xFF;
  tx_buffer_[3] = (can_id >> 8) & 0xFF;
  tx_buffer_[4] = (can_id >> 16) & 0xFF;
  tx_buffer_[5] = (can_id >> 24) & 0xFF;
  
  // 数据长度
  tx_buffer_[6] = len;
  
  // 数据
  memcpy(&tx_buffer_[7], data, len);
  
  // 发送
  ssize_t n = write(fd_, tx_buffer_, 30);
  return (n == 30);
}

bool CANBus::receive(CANFrame& frame) {
  if (fd_ < 0) {
    return false;
  }
  
  // 非阻塞读取16字节
  ssize_t n = read(fd_, rx_buffer_, 16);
  if (n != 16) {
    return false;
  }
  
  // 解析接收帧
  // 格式参考 Python 实现
  if (rx_buffer_[0] != 0xAA || rx_buffer_[1] != 0x55) {
    return false;
  }
  
  // CAN ID
  frame.can_id = rx_buffer_[2] | (rx_buffer_[3] << 8) | 
                 (rx_buffer_[4] << 16) | (rx_buffer_[5] << 24);
  
  // 数据长度
  frame.len = rx_buffer_[6];
  if (frame.len > 8) {
    return false;
  }
  
  // 数据
  memcpy(frame.data, &rx_buffer_[7], frame.len);
  
  return true;
}

// ========== CANNetwork 实现 ==========

CANNetwork::CANNetwork()
  : running_(false)
{}

CANNetwork::~CANNetwork() {
  stop();
  closeAll();
}

bool CANNetwork::addBus(uint8_t bus_id, const std::string& port, int baudrate) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto bus = std::make_shared<CANBus>(bus_id, port, baudrate);
  if (!bus->open()) {
    return false;
  }
  
  buses_.push_back(bus);
  return true;
}

bool CANNetwork::send(uint8_t bus_id, uint32_t can_id, const uint8_t* data, size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& bus : buses_) {
    if (bus->getBusId() == bus_id) {
      return bus->send(can_id, data, len);
    }
  }
  
  return false;
}

void CANNetwork::setCallback(CANRxCallback callback) {
  rx_callback_ = callback;
}

void CANNetwork::start() {
  if (running_) {
    return;
  }
  
  running_ = true;
  rx_thread_ = std::thread(&CANNetwork::receiveLoop, this);
}

void CANNetwork::stop() {
  if (!running_) {
    return;
  }
  
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void CANNetwork::closeAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& bus : buses_) {
    bus->close();
  }
  buses_.clear();
}

void CANNetwork::receiveLoop() {
  while (running_) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (auto& bus : buses_) {
      CANFrame frame;
      if (bus->receive(frame)) {
        if (rx_callback_) {
          rx_callback_(bus->getBusId(), frame.can_id, frame.data, frame.len);
        }
      }
    }
    
    // 短暂休眠避免 CPU 占用过高
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

} // namespace motor_control
