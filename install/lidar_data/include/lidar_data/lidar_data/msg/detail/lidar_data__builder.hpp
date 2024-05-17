// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_data:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__BUILDER_HPP_
#define LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_data/msg/detail/lidar_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_data
{

namespace msg
{

namespace builder
{

class Init_LidarData_length
{
public:
  explicit Init_LidarData_length(::lidar_data::msg::LidarData & msg)
  : msg_(msg)
  {}
  ::lidar_data::msg::LidarData length(::lidar_data::msg::LidarData::_length_type arg)
  {
    msg_.length = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_data::msg::LidarData msg_;
};

class Init_LidarData_data
{
public:
  explicit Init_LidarData_data(::lidar_data::msg::LidarData & msg)
  : msg_(msg)
  {}
  Init_LidarData_length data(::lidar_data::msg::LidarData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_LidarData_length(msg_);
  }

private:
  ::lidar_data::msg::LidarData msg_;
};

class Init_LidarData_header
{
public:
  Init_LidarData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarData_data header(::lidar_data::msg::LidarData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LidarData_data(msg_);
  }

private:
  ::lidar_data::msg::LidarData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_data::msg::LidarData>()
{
  return lidar_data::msg::builder::Init_LidarData_header();
}

}  // namespace lidar_data

#endif  // LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__BUILDER_HPP_
