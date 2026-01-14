// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:srv/MotorSwitchMode.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SWITCH_MODE__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SWITCH_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/srv/detail/motor_switch_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace srv
{

namespace builder
{

class Init_MotorSwitchMode_Request_mode
{
public:
  explicit Init_MotorSwitchMode_Request_mode(::motor_control_ros2::srv::MotorSwitchMode_Request & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::srv::MotorSwitchMode_Request mode(::motor_control_ros2::srv::MotorSwitchMode_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorSwitchMode_Request msg_;
};

class Init_MotorSwitchMode_Request_motor_name
{
public:
  Init_MotorSwitchMode_Request_motor_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSwitchMode_Request_mode motor_name(::motor_control_ros2::srv::MotorSwitchMode_Request::_motor_name_type arg)
  {
    msg_.motor_name = std::move(arg);
    return Init_MotorSwitchMode_Request_mode(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorSwitchMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::srv::MotorSwitchMode_Request>()
{
  return motor_control_ros2::srv::builder::Init_MotorSwitchMode_Request_motor_name();
}

}  // namespace motor_control_ros2


namespace motor_control_ros2
{

namespace srv
{

namespace builder
{

class Init_MotorSwitchMode_Response_message
{
public:
  explicit Init_MotorSwitchMode_Response_message(::motor_control_ros2::srv::MotorSwitchMode_Response & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::srv::MotorSwitchMode_Response message(::motor_control_ros2::srv::MotorSwitchMode_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorSwitchMode_Response msg_;
};

class Init_MotorSwitchMode_Response_success
{
public:
  Init_MotorSwitchMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSwitchMode_Response_message success(::motor_control_ros2::srv::MotorSwitchMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MotorSwitchMode_Response_message(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorSwitchMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::srv::MotorSwitchMode_Response>()
{
  return motor_control_ros2::srv::builder::Init_MotorSwitchMode_Response_success();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SWITCH_MODE__BUILDER_HPP_
