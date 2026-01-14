// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_control_ros2:srv/MotorSetZero.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SET_ZERO__TRAITS_HPP_
#define MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SET_ZERO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_control_ros2/srv/detail/motor_set_zero__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace motor_control_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotorSetZero_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: motor_names
  {
    if (msg.motor_names.size() == 0) {
      out << "motor_names: []";
    } else {
      out << "motor_names: [";
      size_t pending_items = msg.motor_names.size();
      for (auto item : msg.motor_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorSetZero_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motor_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_names.size() == 0) {
      out << "motor_names: []\n";
    } else {
      out << "motor_names:\n";
      for (auto item : msg.motor_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSetZero_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace motor_control_ros2

namespace rosidl_generator_traits
{

[[deprecated("use motor_control_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_control_ros2::srv::MotorSetZero_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_ros2::srv::MotorSetZero_Request & msg)
{
  return motor_control_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_ros2::srv::MotorSetZero_Request>()
{
  return "motor_control_ros2::srv::MotorSetZero_Request";
}

template<>
inline const char * name<motor_control_ros2::srv::MotorSetZero_Request>()
{
  return "motor_control_ros2/srv/MotorSetZero_Request";
}

template<>
struct has_fixed_size<motor_control_ros2::srv::MotorSetZero_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_control_ros2::srv::MotorSetZero_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_control_ros2::srv::MotorSetZero_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace motor_control_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotorSetZero_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorSetZero_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSetZero_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace motor_control_ros2

namespace rosidl_generator_traits
{

[[deprecated("use motor_control_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_control_ros2::srv::MotorSetZero_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_ros2::srv::MotorSetZero_Response & msg)
{
  return motor_control_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_ros2::srv::MotorSetZero_Response>()
{
  return "motor_control_ros2::srv::MotorSetZero_Response";
}

template<>
inline const char * name<motor_control_ros2::srv::MotorSetZero_Response>()
{
  return "motor_control_ros2/srv/MotorSetZero_Response";
}

template<>
struct has_fixed_size<motor_control_ros2::srv::MotorSetZero_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_control_ros2::srv::MotorSetZero_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_control_ros2::srv::MotorSetZero_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<motor_control_ros2::srv::MotorSetZero>()
{
  return "motor_control_ros2::srv::MotorSetZero";
}

template<>
inline const char * name<motor_control_ros2::srv::MotorSetZero>()
{
  return "motor_control_ros2/srv/MotorSetZero";
}

template<>
struct has_fixed_size<motor_control_ros2::srv::MotorSetZero>
  : std::integral_constant<
    bool,
    has_fixed_size<motor_control_ros2::srv::MotorSetZero_Request>::value &&
    has_fixed_size<motor_control_ros2::srv::MotorSetZero_Response>::value
  >
{
};

template<>
struct has_bounded_size<motor_control_ros2::srv::MotorSetZero>
  : std::integral_constant<
    bool,
    has_bounded_size<motor_control_ros2::srv::MotorSetZero_Request>::value &&
    has_bounded_size<motor_control_ros2::srv::MotorSetZero_Response>::value
  >
{
};

template<>
struct is_service<motor_control_ros2::srv::MotorSetZero>
  : std::true_type
{
};

template<>
struct is_service_request<motor_control_ros2::srv::MotorSetZero_Request>
  : std::true_type
{
};

template<>
struct is_service_response<motor_control_ros2::srv::MotorSetZero_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_SET_ZERO__TRAITS_HPP_
