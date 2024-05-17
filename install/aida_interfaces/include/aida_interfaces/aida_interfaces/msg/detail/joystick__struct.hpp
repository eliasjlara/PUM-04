// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aida_interfaces:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__STRUCT_HPP_
#define AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__aida_interfaces__msg__Joystick __attribute__((deprecated))
#else
# define DEPRECATED__aida_interfaces__msg__Joystick __declspec(deprecated)
#endif

namespace aida_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joystick_
{
  using Type = Joystick_<ContainerAllocator>;

  explicit Joystick_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0ul;
      this->y = 0ul;
    }
  }

  explicit Joystick_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0ul;
      this->y = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    uint32_t;
  _x_type x;
  using _y_type =
    uint32_t;
  _y_type y;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const uint32_t & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const uint32_t & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aida_interfaces::msg::Joystick_<ContainerAllocator> *;
  using ConstRawPtr =
    const aida_interfaces::msg::Joystick_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aida_interfaces::msg::Joystick_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aida_interfaces::msg::Joystick_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aida_interfaces__msg__Joystick
    std::shared_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aida_interfaces__msg__Joystick
    std::shared_ptr<aida_interfaces::msg::Joystick_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joystick_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joystick_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joystick_

// alias to use template instance with default allocator
using Joystick =
  aida_interfaces::msg::Joystick_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace aida_interfaces

#endif  // AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__STRUCT_HPP_
