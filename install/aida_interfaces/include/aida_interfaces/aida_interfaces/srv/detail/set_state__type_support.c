// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aida_interfaces:srv/SetState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aida_interfaces/srv/detail/set_state__rosidl_typesupport_introspection_c.h"
#include "aida_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aida_interfaces/srv/detail/set_state__functions.h"
#include "aida_interfaces/srv/detail/set_state__struct.h"


// Include directives for member types
// Member `desired_state`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aida_interfaces__srv__SetState_Request__init(message_memory);
}

void aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_fini_function(void * message_memory)
{
  aida_interfaces__srv__SetState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_member_array[1] = {
  {
    "desired_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aida_interfaces__srv__SetState_Request, desired_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_members = {
  "aida_interfaces__srv",  // message namespace
  "SetState_Request",  // message name
  1,  // number of fields
  sizeof(aida_interfaces__srv__SetState_Request),
  aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_member_array,  // message members
  aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_type_support_handle = {
  0,
  &aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aida_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState_Request)() {
  if (!aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_type_support_handle.typesupport_identifier) {
    aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aida_interfaces__srv__SetState_Request__rosidl_typesupport_introspection_c__SetState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "aida_interfaces/srv/detail/set_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "aida_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "aida_interfaces/srv/detail/set_state__functions.h"
// already included above
// #include "aida_interfaces/srv/detail/set_state__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aida_interfaces__srv__SetState_Response__init(message_memory);
}

void aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_fini_function(void * message_memory)
{
  aida_interfaces__srv__SetState_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aida_interfaces__srv__SetState_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aida_interfaces__srv__SetState_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_members = {
  "aida_interfaces__srv",  // message namespace
  "SetState_Response",  // message name
  2,  // number of fields
  sizeof(aida_interfaces__srv__SetState_Response),
  aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_member_array,  // message members
  aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_type_support_handle = {
  0,
  &aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aida_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState_Response)() {
  if (!aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_type_support_handle.typesupport_identifier) {
    aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aida_interfaces__srv__SetState_Response__rosidl_typesupport_introspection_c__SetState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "aida_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "aida_interfaces/srv/detail/set_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_members = {
  "aida_interfaces__srv",  // service namespace
  "SetState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_Request_message_type_support_handle,
  NULL  // response message
  // aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_Response_message_type_support_handle
};

static rosidl_service_type_support_t aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_type_support_handle = {
  0,
  &aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aida_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState)() {
  if (!aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_type_support_handle.typesupport_identifier) {
    aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aida_interfaces, srv, SetState_Response)()->data;
  }

  return &aida_interfaces__srv__detail__set_state__rosidl_typesupport_introspection_c__SetState_service_type_support_handle;
}
