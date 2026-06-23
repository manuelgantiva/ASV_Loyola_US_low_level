// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asv_interfaces:msg/XbeeObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__TRAITS_HPP_
#define ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asv_interfaces/msg/detail/xbee_observer__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'states'
#include "asv_interfaces/msg/detail/state_neighbor__traits.hpp"

namespace asv_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const XbeeObserver & msg,
  std::ostream & out)
{
  out << "{";
  // member: counter
  {
    out << "counter: ";
    rosidl_generator_traits::value_to_yaml(msg.counter, out);
    out << ", ";
  }

  // member: states
  {
    if (msg.states.size() == 0) {
      out << "states: []";
    } else {
      out << "states: [";
      size_t pending_items = msg.states.size();
      for (auto item : msg.states) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const XbeeObserver & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: counter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "counter: ";
    rosidl_generator_traits::value_to_yaml(msg.counter, out);
    out << "\n";
  }

  // member: states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.states.size() == 0) {
      out << "states: []\n";
    } else {
      out << "states:\n";
      for (auto item : msg.states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const XbeeObserver & msg, bool use_flow_style = false)
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

}  // namespace asv_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use asv_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const asv_interfaces::msg::XbeeObserver & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::msg::XbeeObserver & msg)
{
  return asv_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::msg::XbeeObserver>()
{
  return "asv_interfaces::msg::XbeeObserver";
}

template<>
inline const char * name<asv_interfaces::msg::XbeeObserver>()
{
  return "asv_interfaces/msg/XbeeObserver";
}

template<>
struct has_fixed_size<asv_interfaces::msg::XbeeObserver>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asv_interfaces::msg::XbeeObserver>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asv_interfaces::msg::XbeeObserver>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASV_INTERFACES__MSG__DETAIL__XBEE_OBSERVER__TRAITS_HPP_
