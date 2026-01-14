// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:srv/MotorEnable.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__motor_control_ros2__srv__MotorEnable_Request __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__srv__MotorEnable_Request __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorEnable_Request_
{
  using Type = MotorEnable_Request_<ContainerAllocator>;

  explicit MotorEnable_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
    }
  }

  explicit MotorEnable_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
    }
  }

  // field types and members
  using _motor_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _motor_names_type motor_names;
  using _enable_type =
    bool;
  _enable_type enable;

  // setters for named parameter idiom
  Type & set__motor_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->motor_names = _arg;
    return *this;
  }
  Type & set__enable(
    const bool & _arg)
  {
    this->enable = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__srv__MotorEnable_Request
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__srv__MotorEnable_Request
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorEnable_Request_ & other) const
  {
    if (this->motor_names != other.motor_names) {
      return false;
    }
    if (this->enable != other.enable) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorEnable_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorEnable_Request_

// alias to use template instance with default allocator
using MotorEnable_Request =
  motor_control_ros2::srv::MotorEnable_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace motor_control_ros2


#ifndef _WIN32
# define DEPRECATED__motor_control_ros2__srv__MotorEnable_Response __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__srv__MotorEnable_Response __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorEnable_Response_
{
  using Type = MotorEnable_Response_<ContainerAllocator>;

  explicit MotorEnable_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit MotorEnable_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__srv__MotorEnable_Response
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__srv__MotorEnable_Response
    std::shared_ptr<motor_control_ros2::srv::MotorEnable_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorEnable_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorEnable_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorEnable_Response_

// alias to use template instance with default allocator
using MotorEnable_Response =
  motor_control_ros2::srv::MotorEnable_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace motor_control_ros2

namespace motor_control_ros2
{

namespace srv
{

struct MotorEnable
{
  using Request = motor_control_ros2::srv::MotorEnable_Request;
  using Response = motor_control_ros2::srv::MotorEnable_Response;
};

}  // namespace srv

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__SRV__DETAIL__MOTOR_ENABLE__STRUCT_HPP_
