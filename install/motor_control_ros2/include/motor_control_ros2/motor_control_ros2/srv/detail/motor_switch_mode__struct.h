// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:srv/MotorSwitchMode.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SWITCH_MODE__STRUCT_H_
#define MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SWITCH_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorSwitchMode in the package motor_control_ros2.
typedef struct motor_control_ros2__srv__MotorSwitchMode_Request
{
  rosidl_runtime_c__String motor_name;
  /// 电机模式（具体值取决于电机类型）
  uint8_t mode;
} motor_control_ros2__srv__MotorSwitchMode_Request;

// Struct for a sequence of motor_control_ros2__srv__MotorSwitchMode_Request.
typedef struct motor_control_ros2__srv__MotorSwitchMode_Request__Sequence
{
  motor_control_ros2__srv__MotorSwitchMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__srv__MotorSwitchMode_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorSwitchMode in the package motor_control_ros2.
typedef struct motor_control_ros2__srv__MotorSwitchMode_Response
{
  bool success;
  rosidl_runtime_c__String message;
} motor_control_ros2__srv__MotorSwitchMode_Response;

// Struct for a sequence of motor_control_ros2__srv__MotorSwitchMode_Response.
typedef struct motor_control_ros2__srv__MotorSwitchMode_Response__Sequence
{
  motor_control_ros2__srv__MotorSwitchMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__srv__MotorSwitchMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SWITCH_MODE__STRUCT_H_
