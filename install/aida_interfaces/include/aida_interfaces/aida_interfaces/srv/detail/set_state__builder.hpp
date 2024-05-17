// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aida_interfaces:srv/SetState.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__SRV__DETAIL__SET_STATE__BUILDER_HPP_
#define AIDA_INTERFACES__SRV__DETAIL__SET_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aida_interfaces/srv/detail/set_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aida_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetState_Request_desired_state
{
public:
  Init_SetState_Request_desired_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::aida_interfaces::srv::SetState_Request desired_state(::aida_interfaces::srv::SetState_Request::_desired_state_type arg)
  {
    msg_.desired_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aida_interfaces::srv::SetState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aida_interfaces::srv::SetState_Request>()
{
  return aida_interfaces::srv::builder::Init_SetState_Request_desired_state();
}

}  // namespace aida_interfaces


namespace aida_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetState_Response_message
{
public:
  explicit Init_SetState_Response_message(::aida_interfaces::srv::SetState_Response & msg)
  : msg_(msg)
  {}
  ::aida_interfaces::srv::SetState_Response message(::aida_interfaces::srv::SetState_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aida_interfaces::srv::SetState_Response msg_;
};

class Init_SetState_Response_success
{
public:
  Init_SetState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetState_Response_message success(::aida_interfaces::srv::SetState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetState_Response_message(msg_);
  }

private:
  ::aida_interfaces::srv::SetState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::aida_interfaces::srv::SetState_Response>()
{
  return aida_interfaces::srv::builder::Init_SetState_Response_success();
}

}  // namespace aida_interfaces

#endif  // AIDA_INTERFACES__SRV__DETAIL__SET_STATE__BUILDER_HPP_
