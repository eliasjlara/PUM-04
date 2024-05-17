// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from audio_data:msg/AudioData.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__BUILDER_HPP_
#define AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "audio_data/msg/detail/audio_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace audio_data
{

namespace msg
{

namespace builder
{

class Init_AudioData_samples
{
public:
  explicit Init_AudioData_samples(::audio_data::msg::AudioData & msg)
  : msg_(msg)
  {}
  ::audio_data::msg::AudioData samples(::audio_data::msg::AudioData::_samples_type arg)
  {
    msg_.samples = std::move(arg);
    return std::move(msg_);
  }

private:
  ::audio_data::msg::AudioData msg_;
};

class Init_AudioData_channels
{
public:
  explicit Init_AudioData_channels(::audio_data::msg::AudioData & msg)
  : msg_(msg)
  {}
  Init_AudioData_samples channels(::audio_data::msg::AudioData::_channels_type arg)
  {
    msg_.channels = std::move(arg);
    return Init_AudioData_samples(msg_);
  }

private:
  ::audio_data::msg::AudioData msg_;
};

class Init_AudioData_sample_rate
{
public:
  explicit Init_AudioData_sample_rate(::audio_data::msg::AudioData & msg)
  : msg_(msg)
  {}
  Init_AudioData_channels sample_rate(::audio_data::msg::AudioData::_sample_rate_type arg)
  {
    msg_.sample_rate = std::move(arg);
    return Init_AudioData_channels(msg_);
  }

private:
  ::audio_data::msg::AudioData msg_;
};

class Init_AudioData_data
{
public:
  explicit Init_AudioData_data(::audio_data::msg::AudioData & msg)
  : msg_(msg)
  {}
  Init_AudioData_sample_rate data(::audio_data::msg::AudioData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_AudioData_sample_rate(msg_);
  }

private:
  ::audio_data::msg::AudioData msg_;
};

class Init_AudioData_header
{
public:
  Init_AudioData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AudioData_data header(::audio_data::msg::AudioData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AudioData_data(msg_);
  }

private:
  ::audio_data::msg::AudioData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::audio_data::msg::AudioData>()
{
  return audio_data::msg::builder::Init_AudioData_header();
}

}  // namespace audio_data

#endif  // AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__BUILDER_HPP_
