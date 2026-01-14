#ifndef MOTOR_CONTROL_ROS2__SERIAL_PORT_HPP_
#define MOTOR_CONTROL_ROS2__SERIAL_PORT_HPP_

#include <string>
#include <vector>
#include <mutex>

namespace motor_control {

/**
 * @brief 通用串口通信类
 * 
 * 用于宇树电机等直接通过串口通信的设备。
 * 支持 RS485 模式需要的流控设置（如果硬件需要）。
 */
class SerialPort {
public:
  /**
   * @brief 构造函数
   * @param port_name 串口设备路径 (e.g., /dev/ttyUSB0)
   * @param baudrate 波特率 (默认 4800000 用于宇树电机)
   */
  SerialPort(const std::string& port_name, int baudrate = 4800000);
  ~SerialPort();
  
  /**
   * @brief 打开串口
   * @return 成功返回 true
   */
  bool open();
  
  /**
   * @brief 关闭串口
   */
  void close();
  
  /**
   * @brief 发送数据
   * @param data 数据指针
   * @param len 数据长度
   * @return 发送的字节数，失败返回 -1
   */
  int send(const uint8_t* data, size_t len);
  
  /**
   * @brief 接收数据
   * @param buffer 接收缓冲区
   * @param max_len 最大接收长度
   * @return 接收到的字节数，失败返回 -1
   */
  int receive(uint8_t* buffer, size_t max_len);
  
  /**
   * @brief 是否已打开
   */
  bool isOpen() const { return fd_ >= 0; }
  
  /**
   * @brief 获取端口名
   */
  std::string getPortName() const { return port_name_; }

private:
  std::string port_name_;
  int baudrate_;
  int fd_;
  std::mutex mutex_;
  
  bool configurePort();
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__SERIAL_PORT_HPP_
