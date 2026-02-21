#ifndef MOTOR_CONTROL_ROS2__SERIAL_PORT_MANAGER_HPP_
#define MOTOR_CONTROL_ROS2__SERIAL_PORT_MANAGER_HPP_

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include "serialPort/SerialPort.h"

namespace motor_control {

/**
 * @brief 全局 SerialPort 管理器
 * 
 * 确保同一串口不会被多个对象重复打开，
 * 避免竞态条件和文件描述符泄漏。
 * 
 * 使用方式：
 * SerialPort* port = SerialPortManager::getInstance()->getPort("/dev/ttyUSB0");
 */
class SerialPortManager {
public:
  /**
   * @brief 获取单例实例
   */
  static SerialPortManager* getInstance() {
    static SerialPortManager instance;
    return &instance;
  }

  /**
   * @brief 获取或创建指定串口的 SerialPort 对象
   * @param port_name 串口名称 (e.g. "/dev/ttyUSB0")
   * @param baudrate 波特率 (default 4000000)
   * @param timeout_us 接收超时时间 (微秒，default 100000 = 100ms for ROS2 threading)
   * @return SerialPort 指针，如果创建失败返回 nullptr
   */
  SerialPort* getPort(const std::string& port_name,
                      uint32_t baudrate = 4000000,
                      size_t timeout_us = 100000) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = ports_.find(port_name);
    if (it != ports_.end()) {
      return it->second.get();
    }
    
    // 创建新的 SerialPort
    try {
      auto port = std::make_unique<SerialPort>(
        port_name.c_str(),
        16,  // recvLength
        baudrate,
        timeout_us,
        BlockYN::YES  // 使用阻塞模式以避免 ROS2 线程中的非阻塞超时问题
      );
      SerialPort* ptr = port.get();
      ports_[port_name] = std::move(port);
      return ptr;
    } catch (const std::exception& e) {
      std::cerr << "[SerialPortManager] Failed to open " << port_name 
                << ": " << e.what() << std::endl;
      return nullptr;
    }
  }

  /**
   * @brief 关闭指定串口
   */
  void closePort(const std::string& port_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    ports_.erase(port_name);
  }

  /**
   * @brief 关闭所有串口
   */
  void closeAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    ports_.clear();
  }

private:
  SerialPortManager() = default;
  ~SerialPortManager() { closeAll(); }
  
  // 禁止拷贝和移动
  SerialPortManager(const SerialPortManager&) = delete;
  SerialPortManager& operator=(const SerialPortManager&) = delete;

  std::map<std::string, std::unique_ptr<SerialPort>> ports_;
  std::mutex mutex_;
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__SERIAL_PORT_MANAGER_HPP_
