// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aida_interfaces:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__BUILDER_HPP_
#define AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aida_interfaces/msg/detail/joystick__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aida_interfaces
{

namespace msg
{

namespace builder
{

class Init_Joystick_y
{
public:
  explicit Init_Joystick_y(::aida_interfaces::msg::Joystick & msg)
  : msg_(msg)
  {}
  ::aida_interfaces::msg::Joystick y(::aida_interfaces::msg::Joystick::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aida_interfaces::msg::Joystick msg_;
};

class Init_Joystick_x
{
public:
  explicit Init_Joystick_x(::aida_interfaces::msg::Joystick & msg)
  : msg_(msg)
  {}
  Init_Joystick_y x(::aida_interfaces::msg::Joystick::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Joystick_y(msg_);
  }

private:
  ::aida_interfaces::msg::Joystick msg_;
};

class Init_Joystick_header
{
public:
  Init_Joystick_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Joystick_x header(::aida_interfaces::msg::Joystick::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Joystick_x(msg_);
  }

private:
  ::aida_interfaces::msg::Joystick msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aida_interfaces::msg::Joystick>()
{
  return aida_interfaces::msg::builder::Init_Joystick_header();
}

}  // namespace aida_interfaces

#endif  // AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__BUILDER_HPP_
