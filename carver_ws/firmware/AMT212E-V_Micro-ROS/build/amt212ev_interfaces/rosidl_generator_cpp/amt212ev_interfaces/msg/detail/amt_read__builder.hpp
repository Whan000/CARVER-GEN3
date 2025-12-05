// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#ifndef AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__BUILDER_HPP_
#define AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "amt212ev_interfaces/msg/detail/amt_read__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace amt212ev_interfaces
{

namespace msg
{

namespace builder
{

class Init_AmtRead_radps
{
public:
  explicit Init_AmtRead_radps(::amt212ev_interfaces::msg::AmtRead & msg)
  : msg_(msg)
  {}
  ::amt212ev_interfaces::msg::AmtRead radps(::amt212ev_interfaces::msg::AmtRead::_radps_type arg)
  {
    msg_.radps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::amt212ev_interfaces::msg::AmtRead msg_;
};

class Init_AmtRead_rads
{
public:
  Init_AmtRead_rads()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AmtRead_radps rads(::amt212ev_interfaces::msg::AmtRead::_rads_type arg)
  {
    msg_.rads = std::move(arg);
    return Init_AmtRead_radps(msg_);
  }

private:
  ::amt212ev_interfaces::msg::AmtRead msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::amt212ev_interfaces::msg::AmtRead>()
{
  return amt212ev_interfaces::msg::builder::Init_AmtRead_rads();
}

}  // namespace amt212ev_interfaces

#endif  // AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__BUILDER_HPP_
