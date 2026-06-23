// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asv_interfaces:srv/SetObs.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__SRV__DETAIL__SET_OBS__TRAITS_HPP_
#define ASV_INTERFACES__SRV__DETAIL__SET_OBS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asv_interfaces/srv/detail/set_obs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace asv_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetObs_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: eso_mode
  {
    out << "eso_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.eso_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetObs_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: eso_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "eso_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.eso_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetObs_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace asv_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use asv_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const asv_interfaces::srv::SetObs_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::srv::SetObs_Request & msg)
{
  return asv_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::srv::SetObs_Request>()
{
  return "asv_interfaces::srv::SetObs_Request";
}

template<>
inline const char * name<asv_interfaces::srv::SetObs_Request>()
{
  return "asv_interfaces/srv/SetObs_Request";
}

template<>
struct has_fixed_size<asv_interfaces::srv::SetObs_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<asv_interfaces::srv::SetObs_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<asv_interfaces::srv::SetObs_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace asv_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetObs_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetObs_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetObs_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace asv_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use asv_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const asv_interfaces::srv::SetObs_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::srv::SetObs_Response & msg)
{
  return asv_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::srv::SetObs_Response>()
{
  return "asv_interfaces::srv::SetObs_Response";
}

template<>
inline const char * name<asv_interfaces::srv::SetObs_Response>()
{
  return "asv_interfaces/srv/SetObs_Response";
}

template<>
struct has_fixed_size<asv_interfaces::srv::SetObs_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<asv_interfaces::srv::SetObs_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<asv_interfaces::srv::SetObs_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<asv_interfaces::srv::SetObs>()
{
  return "asv_interfaces::srv::SetObs";
}

template<>
inline const char * name<asv_interfaces::srv::SetObs>()
{
  return "asv_interfaces/srv/SetObs";
}

template<>
struct has_fixed_size<asv_interfaces::srv::SetObs>
  : std::integral_constant<
    bool,
    has_fixed_size<asv_interfaces::srv::SetObs_Request>::value &&
    has_fixed_size<asv_interfaces::srv::SetObs_Response>::value
  >
{
};

template<>
struct has_bounded_size<asv_interfaces::srv::SetObs>
  : std::integral_constant<
    bool,
    has_bounded_size<asv_interfaces::srv::SetObs_Request>::value &&
    has_bounded_size<asv_interfaces::srv::SetObs_Response>::value
  >
{
};

template<>
struct is_service<asv_interfaces::srv::SetObs>
  : std::true_type
{
};

template<>
struct is_service_request<asv_interfaces::srv::SetObs_Request>
  : std::true_type
{
};

template<>
struct is_service_response<asv_interfaces::srv::SetObs_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ASV_INTERFACES__SRV__DETAIL__SET_OBS__TRAITS_HPP_
