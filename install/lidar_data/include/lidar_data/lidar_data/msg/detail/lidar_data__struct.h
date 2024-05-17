// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_data:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__STRUCT_H_
#define LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__STRUCT_H_

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

/// Struct defined in msg/LidarData in the package lidar_data.
/**
  * LidarData.msg
 */
typedef struct lidar_data__msg__LidarData
{
  /// Header for the message
  std_msgs__msg__Header header;
  /// The lidar data, as a byte array
  rosidl_runtime_c__int32__Sequence data;
  /// The lidar data array lenght
  int32_t length;
} lidar_data__msg__LidarData;

// Struct for a sequence of lidar_data__msg__LidarData.
typedef struct lidar_data__msg__LidarData__Sequence
{
  lidar_data__msg__LidarData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_data__msg__LidarData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_DATA__MSG__DETAIL__LIDAR_DATA__STRUCT_H_
