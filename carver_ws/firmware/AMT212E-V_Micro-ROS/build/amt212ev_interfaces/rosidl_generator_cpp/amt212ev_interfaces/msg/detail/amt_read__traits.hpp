// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#ifndef AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__TRAITS_HPP_
#define AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "amt212ev_interfaces/msg/detail/amt_read__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace amt212ev_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AmtRead & msg,
  std::ostream & out)
{
  out << "{";
  // member: rads
  {
    out << "rads: ";
    rosidl_generator_traits::value_to_yaml(msg.rads, out);
    out << ", ";
  }

  // member: radps
  {
    out << "radps: ";
    rosidl_generator_traits::value_to_yaml(msg.radps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AmtRead & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rads
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rads: ";
    rosidl_generator_traits::value_to_yaml(msg.rads, out);
    out << "\n";
  }

  // member: radps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radps: ";
    rosidl_generator_traits::value_to_yaml(msg.radps, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AmtRead & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace amt212ev_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use amt212ev_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const amt212ev_interfaces::msg::AmtRead & msg,
  std::ostream & out, size_t indentation = 0)
{
  amt212ev_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use amt212ev_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const amt212ev_interfaces::msg::AmtRead & msg)
{
  return amt212ev_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<amt212ev_interfaces::msg::AmtRead>()
{
  return "amt212ev_interfaces::msg::AmtRead";
}

template<>
inline const char * name<amt212ev_interfaces::msg::AmtRead>()
{
  return "amt212ev_interfaces/msg/AmtRead";
}

template<>
struct has_fixed_size<amt212ev_interfaces::msg::AmtRead>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<amt212ev_interfaces::msg::AmtRead>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<amt212ev_interfaces::msg::AmtRead>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__TRAITS_HPP_
