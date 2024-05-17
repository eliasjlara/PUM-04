// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from audio_data:msg/AudioData.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_
#define AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_

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
# define DEPRECATED__audio_data__msg__AudioData __attribute__((deprecated))
#else
# define DEPRECATED__audio_data__msg__AudioData __declspec(deprecated)
#endif

namespace audio_data
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AudioData_
{
  using Type = AudioData_<ContainerAllocator>;

  explicit AudioData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_rate = 0l;
      this->channels = 0l;
      this->samples = 0l;
    }
  }

  explicit AudioData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_rate = 0l;
      this->channels = 0l;
      this->samples = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;
  using _sample_rate_type =
    int32_t;
  _sample_rate_type sample_rate;
  using _channels_type =
    int32_t;
  _channels_type channels;
  using _samples_type =
    int32_t;
  _samples_type samples;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__sample_rate(
    const int32_t & _arg)
  {
    this->sample_rate = _arg;
    return *this;
  }
  Type & set__channels(
    const int32_t & _arg)
  {
    this->channels = _arg;
    return *this;
  }
  Type & set__samples(
    const int32_t & _arg)
  {
    this->samples = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    audio_data::msg::AudioData_<ContainerAllocator> *;
  using ConstRawPtr =
    const audio_data::msg::AudioData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<audio_data::msg::AudioData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<audio_data::msg::AudioData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      audio_data::msg::AudioData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<audio_data::msg::AudioData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      audio_data::msg::AudioData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<audio_data::msg::AudioData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<audio_data::msg::AudioData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<audio_data::msg::AudioData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__audio_data__msg__AudioData
    std::shared_ptr<audio_data::msg::AudioData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__audio_data__msg__AudioData
    std::shared_ptr<audio_data::msg::AudioData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AudioData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->sample_rate != other.sample_rate) {
      return false;
    }
    if (this->channels != other.channels) {
      return false;
    }
    if (this->samples != other.samples) {
      return false;
    }
    return true;
  }
  bool operator!=(const AudioData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AudioData_

// alias to use template instance with default allocator
using AudioData =
  audio_data::msg::AudioData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace audio_data

#endif  // AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__STRUCT_HPP_
