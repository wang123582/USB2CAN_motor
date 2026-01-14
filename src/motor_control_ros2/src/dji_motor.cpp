#include "motor_control_ros2/dji_motor.hpp"
#include <cstring>

namespace motor_control {

DJIMotor::DJIMotor(const std::string& joint_name, MotorType type, 
                   uint8_t motor_id, uint8_t bus_id)
  : MotorBase(joint_name, type, 
              (type == MotorType::DJI_GM3508) ? 19.0f : 1.0f,  // 减速比
              true)  // 编码器在输出轴
  , motor_id_(motor_id)
  , bus_id_(bus_id)
  , target_output_(0)
  , raw_angle_(0)
  , raw_rpm_(0)
  , raw_current_(0)
  , raw_temp_(0)
{
  if (type == MotorType::DJI_GM6020) {
    // GM6020 配置
    feedback_id_ = 0x205 + (motor_id - 1);
    control_id_ = (motor_id <= 4) ? 0x1FF : 0x2FF;
    max_output_ = 30000;  // 电压限幅
  } else if (type == MotorType::DJI_GM3508) {
    // GM3508 配置
    feedback_id_ = 0x201 + (motor_id - 1);
    control_id_ = (motor_id <= 4) ? 0x200 : 0x1FF;
    max_output_ = 16384;  // 电流限幅
  }
}

void DJIMotor::updateFeedback(uint32_t can_id, const uint8_t* data, size_t len) {
  if (can_id != feedback_id_ || len < 8) {
    return;
  }
  
  online_ = true;
  
  // 解析 DJI 反馈帧 (Big Endian)
  // Byte 0-1: Angle (0-8191)
  // Byte 2-3: RPM
  // Byte 4-5: Current
  // Byte 6: Temperature
  raw_angle_ = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  raw_rpm_ = (static_cast<int16_t>(data[2]) << 8) | data[3];
  raw_current_ = (static_cast<int16_t>(data[4]) << 8) | data[5];
  raw_temp_ = data[6];
  
  // 转换为标准单位
  // 位置：0-8191 -> 0-2π 弧度
  position_ = (raw_angle_ / 8191.0) * 2.0 * M_PI;
  
  // 速度：RPM -> 弧度/秒
  velocity_ = (raw_rpm_ / 60.0) * 2.0 * M_PI;
  
  // 力矩：原始电流值（需要根据实际电机转矩常数转换）
  // 这里暂时直接使用电流值
  torque_ = raw_current_;
  
  // 温度
  temperature_ = static_cast<float>(raw_temp_);
}

void DJIMotor::getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) {
  // DJI 电机使用共享帧，不单独发送
  // 返回 len = 0 表示需要拼包
  (void)data; // 消除未使用参数警告
  can_id = control_id_;
  len = 0;
}

void DJIMotor::disable() {
  setOutput(0);
}

void DJIMotor::setOutput(int16_t value) {
  // 限幅
  if (value > max_output_) value = max_output_;
  if (value < -max_output_) value = -max_output_;
  target_output_ = value;
}

void DJIMotor::getControlBytes(uint8_t bytes[2]) const {
  // Big Endian
  bytes[0] = (target_output_ >> 8) & 0xFF;
  bytes[1] = target_output_ & 0xFF;
}

} // namespace motor_control
