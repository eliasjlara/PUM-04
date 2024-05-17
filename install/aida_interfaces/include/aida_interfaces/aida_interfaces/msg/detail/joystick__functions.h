// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from aida_interfaces:msg/Joystick.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__FUNCTIONS_H_
#define AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "aida_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "aida_interfaces/msg/detail/joystick__struct.h"

/// Initialize msg/Joystick message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * aida_interfaces__msg__Joystick
 * )) before or use
 * aida_interfaces__msg__Joystick__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
bool
aida_interfaces__msg__Joystick__init(aida_interfaces__msg__Joystick * msg);

/// Finalize msg/Joystick message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
void
aida_interfaces__msg__Joystick__fini(aida_interfaces__msg__Joystick * msg);

/// Create msg/Joystick message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * aida_interfaces__msg__Joystick__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
aida_interfaces__msg__Joystick *
aida_interfaces__msg__Joystick__create();

/// Destroy msg/Joystick message.
/**
 * It calls
 * aida_interfaces__msg__Joystick__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
void
aida_interfaces__msg__Joystick__destroy(aida_interfaces__msg__Joystick * msg);

/// Check for msg/Joystick message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
bool
aida_interfaces__msg__Joystick__are_equal(const aida_interfaces__msg__Joystick * lhs, const aida_interfaces__msg__Joystick * rhs);

/// Copy a msg/Joystick message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
bool
aida_interfaces__msg__Joystick__copy(
  const aida_interfaces__msg__Joystick * input,
  aida_interfaces__msg__Joystick * output);

/// Initialize array of msg/Joystick messages.
/**
 * It allocates the memory for the number of elements and calls
 * aida_interfaces__msg__Joystick__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
bool
aida_interfaces__msg__Joystick__Sequence__init(aida_interfaces__msg__Joystick__Sequence * array, size_t size);

/// Finalize array of msg/Joystick messages.
/**
 * It calls
 * aida_interfaces__msg__Joystick__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
void
aida_interfaces__msg__Joystick__Sequence__fini(aida_interfaces__msg__Joystick__Sequence * array);

/// Create array of msg/Joystick messages.
/**
 * It allocates the memory for the array and calls
 * aida_interfaces__msg__Joystick__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
aida_interfaces__msg__Joystick__Sequence *
aida_interfaces__msg__Joystick__Sequence__create(size_t size);

/// Destroy array of msg/Joystick messages.
/**
 * It calls
 * aida_interfaces__msg__Joystick__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
void
aida_interfaces__msg__Joystick__Sequence__destroy(aida_interfaces__msg__Joystick__Sequence * array);

/// Check for msg/Joystick message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
bool
aida_interfaces__msg__Joystick__Sequence__are_equal(const aida_interfaces__msg__Joystick__Sequence * lhs, const aida_interfaces__msg__Joystick__Sequence * rhs);

/// Copy an array of msg/Joystick messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_aida_interfaces
bool
aida_interfaces__msg__Joystick__Sequence__copy(
  const aida_interfaces__msg__Joystick__Sequence * input,
  aida_interfaces__msg__Joystick__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AIDA_INTERFACES__MSG__DETAIL__JOYSTICK__FUNCTIONS_H_
