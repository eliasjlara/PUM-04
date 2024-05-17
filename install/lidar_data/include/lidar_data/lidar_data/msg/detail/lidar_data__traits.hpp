// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_data:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__TRAITS_HPP_
#define LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidar_data/msg/detail/lidar_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace lidar_data
{

namespace msg
{

inline void to_flow_style_yaml(
  const LidarData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: length
  {
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LidarData & msg,
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

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LidarData & msg, bool use_flow_style = false)
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

}  // namespace lidar_data

namespace rosidl_generator_traits
{

[[deprecated("use lidar_data::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lidar_data::msg::LidarData & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidar_data::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidar_data::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidar_data::msg::LidarData & msg)
{
  return lidar_data::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidar_data::msg::LidarData>()
{
  return "lidar_data::msg::LidarData";
}

template<>
inline const char * name<lidar_data::msg::LidarData>()
{
  return "lidar_data/msg/LidarData";
}

template<>
struct has_fixed_size<lidar_data::msg::LidarData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_data::msg::LidarData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_data::msg::LidarData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__TRAITS_HPP_
