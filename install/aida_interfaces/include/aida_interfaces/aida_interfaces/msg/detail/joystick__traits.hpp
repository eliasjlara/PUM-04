// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aida_interfaces:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__TRAITS_HPP_
#define AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aida_interfaces/msg/detail/joystick__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace aida_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Joystick & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Joystick & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Joystick & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace aida_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use aida_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aida_interfaces::msg::Joystick & msg,
  std::ostream & out, size_t indentation = 0)
{
  aida_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aida_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const aida_interfaces::msg::Joystick & msg)
{
  return aida_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<aida_interfaces::msg::Joystick>()
{
  return "aida_interfaces::msg::Joystick";
}

template<>
inline const char * name<aida_interfaces::msg::Joystick>()
{
  return "aida_interfaces/msg/Joystick";
}

template<>
struct has_fixed_size<aida_interfaces::msg::Joystick>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<aida_interfaces::msg::Joystick>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<aida_interfaces::msg::Joystick>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__TRAITS_HPP_
