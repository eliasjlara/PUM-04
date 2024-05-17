// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aida_interfaces:srv/SetState.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__SRV__DETAIL__SET_STATE__TRAITS_HPP_
#define AIDA_INTERFACES__SRV__DETAIL__SET_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aida_interfaces/srv/detail/set_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace aida_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: desired_state
  {
    out << "desired_state: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: desired_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_state: ";
    rosidl_generator_traits::value_to_yaml(msg.desired_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetState_Request & msg, bool use_flow_style = false)
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

}  // namespace aida_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use aida_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aida_interfaces::srv::SetState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  aida_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aida_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const aida_interfaces::srv::SetState_Request & msg)
{
  return aida_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<aida_interfaces::srv::SetState_Request>()
{
  return "aida_interfaces::srv::SetState_Request";
}

template<>
inline const char * name<aida_interfaces::srv::SetState_Request>()
{
  return "aida_interfaces/srv/SetState_Request";
}

template<>
struct has_fixed_size<aida_interfaces::srv::SetState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aida_interfaces::srv::SetState_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aida_interfaces::srv::SetState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace aida_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetState_Response & msg,
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
  const SetState_Response & msg,
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

inline std::string to_yaml(const SetState_Response & msg, bool use_flow_style = false)
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

}  // namespace aida_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use aida_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aida_interfaces::srv::SetState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  aida_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aida_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const aida_interfaces::srv::SetState_Response & msg)
{
  return aida_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<aida_interfaces::srv::SetState_Response>()
{
  return "aida_interfaces::srv::SetState_Response";
}

template<>
inline const char * name<aida_interfaces::srv::SetState_Response>()
{
  return "aida_interfaces/srv/SetState_Response";
}

template<>
struct has_fixed_size<aida_interfaces::srv::SetState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aida_interfaces::srv::SetState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aida_interfaces::srv::SetState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<aida_interfaces::srv::SetState>()
{
  return "aida_interfaces::srv::SetState";
}

template<>
inline const char * name<aida_interfaces::srv::SetState>()
{
  return "aida_interfaces/srv/SetState";
}

template<>
struct has_fixed_size<aida_interfaces::srv::SetState>
  : std::integral_constant<
    bool,
    has_fixed_size<aida_interfaces::srv::SetState_Request>::value &&
    has_fixed_size<aida_interfaces::srv::SetState_Response>::value
  >
{
};

template<>
struct has_bounded_size<aida_interfaces::srv::SetState>
  : std::integral_constant<
    bool,
    has_bounded_size<aida_interfaces::srv::SetState_Request>::value &&
    has_bounded_size<aida_interfaces::srv::SetState_Response>::value
  >
{
};

template<>
struct is_service<aida_interfaces::srv::SetState>
  : std::true_type
{
};

template<>
struct is_service_request<aida_interfaces::srv::SetState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<aida_interfaces::srv::SetState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AIDA_INTERFACES__SRV__DETAIL__SET_STATE__TRAITS_HPP_
