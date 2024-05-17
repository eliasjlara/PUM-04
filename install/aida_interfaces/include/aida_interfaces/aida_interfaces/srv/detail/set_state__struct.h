// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aida_interfaces:srv/SetState.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__SRV__DETAIL__SET_STATE__STRUCT_H_
#define AIDA_INTERFACES__SRV__DETAIL__SET_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'desired_state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetState in the package aida_interfaces.
typedef struct aida_interfaces__srv__SetState_Request
{
  /// Desired state to transition to ("active" or "idle")
  rosidl_runtime_c__String desired_state;
} aida_interfaces__srv__SetState_Request;

// Struct for a sequence of aida_interfaces__srv__SetState_Request.
typedef struct aida_interfaces__srv__SetState_Request__Sequence
{
  aida_interfaces__srv__SetState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aida_interfaces__srv__SetState_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetState in the package aida_interfaces.
typedef struct aida_interfaces__srv__SetState_Response
{
  /// Whether the state change was successful
  bool success;
  /// Optional message describing the outcome (e.g., error message)
  rosidl_runtime_c__String message;
} aida_interfaces__srv__SetState_Response;

// Struct for a sequence of aida_interfaces__srv__SetState_Response.
typedef struct aida_interfaces__srv__SetState_Response__Sequence
{
  aida_interfaces__srv__SetState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aida_interfaces__srv__SetState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIDA_INTERFACES__SRV__DETAIL__SET_STATE__STRUCT_H_
