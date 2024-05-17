// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aida_interfaces:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__STRUCT_H_
#define AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__STRUCT_H_

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

/// Struct defined in msg/Joystick in the package aida_interfaces.
/**
  * Joystick.msg
 */
typedef struct aida_interfaces__msg__Joystick
{
  /// Header for the message
  std_msgs__msg__Header header;
  /// The joystick positional data, as x and y
  uint32_t x;
  uint32_t y;
} aida_interfaces__msg__Joystick;

// Struct for a sequence of aida_interfaces__msg__Joystick.
typedef struct aida_interfaces__msg__Joystick__Sequence
{
  aida_interfaces__msg__Joystick * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aida_interfaces__msg__Joystick__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__STRUCT_H_
