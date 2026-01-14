// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:srv/MotorEnable.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/srv/detail/motor_enable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace srv
{

namespace builder
{

class Init_MotorEnable_Request_enable
{
public:
  explicit Init_MotorEnable_Request_enable(::motor_control_ros2::srv::MotorEnable_Request & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::srv::MotorEnable_Request enable(::motor_control_ros2::srv::MotorEnable_Request::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorEnable_Request msg_;
};

class Init_MotorEnable_Request_motor_names
{
public:
  Init_MotorEnable_Request_motor_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorEnable_Request_enable motor_names(::motor_control_ros2::srv::MotorEnable_Request::_motor_names_type arg)
  {
    msg_.motor_names = std::move(arg);
    return Init_MotorEnable_Request_enable(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorEnable_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::srv::MotorEnable_Request>()
{
  return motor_control_ros2::srv::builder::Init_MotorEnable_Request_motor_names();
}

}  // namespace motor_control_ros2


namespace motor_control_ros2
{

namespace srv
{

namespace builder
{

class Init_MotorEnable_Response_message
{
public:
  explicit Init_MotorEnable_Response_message(::motor_control_ros2::srv::MotorEnable_Response & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::srv::MotorEnable_Response message(::motor_control_ros2::srv::MotorEnable_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorEnable_Response msg_;
};

class Init_MotorEnable_Response_success
{
public:
  Init_MotorEnable_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorEnable_Response_message success(::motor_control_ros2::srv::MotorEnable_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MotorEnable_Response_message(msg_);
  }

private:
  ::motor_control_ros2::srv::MotorEnable_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::srv::MotorEnable_Response>()
{
  return motor_control_ros2::srv::builder::Init_MotorEnable_Response_success();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__BUILDER_HPP_
