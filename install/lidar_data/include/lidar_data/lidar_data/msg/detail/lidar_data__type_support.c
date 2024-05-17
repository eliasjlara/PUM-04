// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_data:msg/LidarData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_data/msg/detail/lidar_data__rosidl_typesupport_introspection_c.h"
#include "lidar_data/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_data/msg/detail/lidar_data__functions.h"
#include "lidar_data/msg/detail/lidar_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_data__msg__LidarData__init(message_memory);
}

void lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_fini_function(void * message_memory)
{
  lidar_data__msg__LidarData__fini(message_memory);
}

size_t lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__size_function__LidarData__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__get_const_function__LidarData__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__get_function__LidarData__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__fetch_function__LidarData__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__get_const_function__LidarData__data(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__assign_function__LidarData__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__get_function__LidarData__data(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__resize_function__LidarData__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_data__msg__LidarData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_data__msg__LidarData, data),  // bytes offset in struct
    NULL,  // default value
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__size_function__LidarData__data,  // size() function pointer
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__get_const_function__LidarData__data,  // get_const(index) function pointer
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__get_function__LidarData__data,  // get(index) function pointer
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__fetch_function__LidarData__data,  // fetch(index, &value) function pointer
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__assign_function__LidarData__data,  // assign(index, value) function pointer
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__resize_function__LidarData__data  // resize(index) function pointer
  },
  {
    "length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_data__msg__LidarData, length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_members = {
  "lidar_data__msg",  // message namespace
  "LidarData",  // message name
  3,  // number of fields
  sizeof(lidar_data__msg__LidarData),
  lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_member_array,  // message members
  lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_init_function,  // function to initialize message memory (memory has to be allocated)
  lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_type_support_handle = {
  0,
  &lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_data
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_data, msg, LidarData)() {
  lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_type_support_handle.typesupport_identifier) {
    lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidar_data__msg__LidarData__rosidl_typesupport_introspection_c__LidarData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
