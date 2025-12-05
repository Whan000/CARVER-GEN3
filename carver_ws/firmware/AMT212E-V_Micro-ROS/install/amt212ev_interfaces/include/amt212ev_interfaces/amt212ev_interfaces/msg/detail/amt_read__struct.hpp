// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#ifndef AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__STRUCT_HPP_
#define AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__amt212ev_interfaces__msg__AmtRead __attribute__((deprecated))
#else
# define DEPRECATED__amt212ev_interfaces__msg__AmtRead __declspec(deprecated)
#endif

namespace amt212ev_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AmtRead_
{
  using Type = AmtRead_<ContainerAllocator>;

  explicit AmtRead_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rads = 0.0;
      this->radps = 0.0;
    }
  }

  explicit AmtRead_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rads = 0.0;
      this->radps = 0.0;
    }
  }

  // field types and members
  using _rads_type =
    double;
  _rads_type rads;
  using _radps_type =
    double;
  _radps_type radps;

  // setters for named parameter idiom
  Type & set__rads(
    const double & _arg)
  {
    this->rads = _arg;
    return *this;
  }
  Type & set__radps(
    const double & _arg)
  {
    this->radps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    amt212ev_interfaces::msg::AmtRead_<ContainerAllocator> *;
  using ConstRawPtr =
    const amt212ev_interfaces::msg::AmtRead_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      amt212ev_interfaces::msg::AmtRead_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      amt212ev_interfaces::msg::AmtRead_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__amt212ev_interfaces__msg__AmtRead
    std::shared_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__amt212ev_interfaces__msg__AmtRead
    std::shared_ptr<amt212ev_interfaces::msg::AmtRead_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AmtRead_ & other) const
  {
    if (this->rads != other.rads) {
      return false;
    }
    if (this->radps != other.radps) {
      return false;
    }
    return true;
  }
  bool operator!=(const AmtRead_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AmtRead_

// alias to use template instance with default allocator
using AmtRead =
  amt212ev_interfaces::msg::AmtRead_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace amt212ev_interfaces

#endif  // AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__STRUCT_HPP_
