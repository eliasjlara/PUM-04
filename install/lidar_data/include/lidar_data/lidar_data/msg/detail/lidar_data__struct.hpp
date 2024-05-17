// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_data:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__STRUCT_HPP_
#define LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__STRUCT_HPP_

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
# define DEPRECATED__lidar_data__msg__LidarData __attribute__((deprecated))
#else
# define DEPRECATED__lidar_data__msg__LidarData __declspec(deprecated)
#endif

namespace lidar_data
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarData_
{
  using Type = LidarData_<ContainerAllocator>;

  explicit LidarData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0l;
    }
  }

  explicit LidarData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _data_type data;
  using _length_type =
    int32_t;
  _length_type length;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__length(
    const int32_t & _arg)
  {
    this->length = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_data::msg::LidarData_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_data::msg::LidarData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_data::msg::LidarData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_data::msg::LidarData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_data::msg::LidarData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_data::msg::LidarData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_data::msg::LidarData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_data::msg::LidarData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_data::msg::LidarData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_data::msg::LidarData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_data__msg__LidarData
    std::shared_ptr<lidar_data::msg::LidarData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_data__msg__LidarData
    std::shared_ptr<lidar_data::msg::LidarData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarData_

// alias to use template instance with default allocator
using LidarData =
  lidar_data::msg::LidarData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_data

#endif  // LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__STRUCT_HPP_
