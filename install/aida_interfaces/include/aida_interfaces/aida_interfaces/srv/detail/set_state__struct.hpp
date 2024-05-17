// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aida_interfaces:srv/SetState.idl
// generated code does not contain a copyright notice

#ifndef AIDA_INTERFACES__SRV__DETAIL__SET_STATE__STRUCT_HPP_
#define AIDA_INTERFACES__SRV__DETAIL__SET_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__aida_interfaces__srv__SetState_Request __attribute__((deprecated))
#else
# define DEPRECATED__aida_interfaces__srv__SetState_Request __declspec(deprecated)
#endif

namespace aida_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetState_Request_
{
  using Type = SetState_Request_<ContainerAllocator>;

  explicit SetState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->desired_state = "";
    }
  }

  explicit SetState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : desired_state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->desired_state = "";
    }
  }

  // field types and members
  using _desired_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _desired_state_type desired_state;

  // setters for named parameter idiom
  Type & set__desired_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->desired_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aida_interfaces::srv::SetState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const aida_interfaces::srv::SetState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aida_interfaces::srv::SetState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aida_interfaces::srv::SetState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aida_interfaces__srv__SetState_Request
    std::shared_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aida_interfaces__srv__SetState_Request
    std::shared_ptr<aida_interfaces::srv::SetState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetState_Request_ & other) const
  {
    if (this->desired_state != other.desired_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetState_Request_

// alias to use template instance with default allocator
using SetState_Request =
  aida_interfaces::srv::SetState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace aida_interfaces


#ifndef _WIN32
# define DEPRECATED__aida_interfaces__srv__SetState_Response __attribute__((deprecated))
#else
# define DEPRECATED__aida_interfaces__srv__SetState_Response __declspec(deprecated)
#endif

namespace aida_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetState_Response_
{
  using Type = SetState_Response_<ContainerAllocator>;

  explicit SetState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aida_interfaces::srv::SetState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const aida_interfaces::srv::SetState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aida_interfaces::srv::SetState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aida_interfaces::srv::SetState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aida_interfaces__srv__SetState_Response
    std::shared_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aida_interfaces__srv__SetState_Response
    std::shared_ptr<aida_interfaces::srv::SetState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetState_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetState_Response_

// alias to use template instance with default allocator
using SetState_Response =
  aida_interfaces::srv::SetState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace aida_interfaces

namespace aida_interfaces
{

namespace srv
{

struct SetState
{
  using Request = aida_interfaces::srv::SetState_Request;
  using Response = aida_interfaces::srv::SetState_Response;
};

}  // namespace srv

}  // namespace aida_interfaces

#endif  // AIDA_INTERFACES__SRV__DETAIL__SET_STATE__STRUCT_HPP_
