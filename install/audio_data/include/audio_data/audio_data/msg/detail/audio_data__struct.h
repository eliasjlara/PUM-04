// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from audio_data:msg/AudioData.idl
// generated code does not contain a copyright notice

#ifndef AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__STRUCT_H_
#define AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AudioData in the package audio_data.
/**
  * AudioData.msg
 */
typedef struct audio_data__msg__AudioData
{
  /// Header for the message
  std_msgs__msg__Header header;
  /// The audio data, as a byte array
  rosidl_runtime_c__uint8__Sequence data;
  /// The sample rate of the audio data
  int32_t sample_rate;
  /// The number of channels in the audio data
  int32_t channels;
  /// The audio sample count
  int32_t samples;
} audio_data__msg__AudioData;

// Struct for a sequence of audio_data__msg__AudioData.
typedef struct audio_data__msg__AudioData__Sequence
{
  audio_data__msg__AudioData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} audio_data__msg__AudioData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_DATA__MSG__DETAIL__AUDIO_DATA__STRUCT_H_
