// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:srv/MotorEnable.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__STRUCT_H_
#define MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_names'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorEnable in the package motor_control_ros2.
typedef struct motor_control_ros2__srv__MotorEnable_Request
{
  /// 空数组 = 所有电机
  rosidl_runtime_c__String__Sequence motor_names;
  /// true=使能, false=失能
  bool enable;
} motor_control_ros2__srv__MotorEnable_Request;

// Struct for a sequence of motor_control_ros2__srv__MotorEnable_Request.
typedef struct motor_control_ros2__srv__MotorEnable_Request__Sequence
{
  motor_control_ros2__srv__MotorEnable_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__srv__MotorEnable_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorEnable in the package motor_control_ros2.
typedef struct motor_control_ros2__srv__MotorEnable_Response
{
  bool success;
  rosidl_runtime_c__String message;
} motor_control_ros2__srv__MotorEnable_Response;

// Struct for a sequence of motor_control_ros2__srv__MotorEnable_Response.
typedef struct motor_control_ros2__srv__MotorEnable_Response__Sequence
{
  motor_control_ros2__srv__MotorEnable_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__srv__MotorEnable_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__STRUCT_H_
