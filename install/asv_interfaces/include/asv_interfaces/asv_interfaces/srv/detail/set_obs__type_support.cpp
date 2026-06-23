// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from asv_interfaces:srv/SetObs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "asv_interfaces/srv/detail/set_obs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace asv_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetObs_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asv_interfaces::srv::SetObs_Request(_init);
}

void SetObs_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asv_interfaces::srv::SetObs_Request *>(message_memory);
  typed_message->~SetObs_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetObs_Request_message_member_array[1] = {
  {
    "eso_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::srv::SetObs_Request, eso_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetObs_Request_message_members = {
  "asv_interfaces::srv",  // message namespace
  "SetObs_Request",  // message name
  1,  // number of fields
  sizeof(asv_interfaces::srv::SetObs_Request),
  SetObs_Request_message_member_array,  // message members
  SetObs_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetObs_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetObs_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetObs_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace asv_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<asv_interfaces::srv::SetObs_Request>()
{
  return &::asv_interfaces::srv::rosidl_typesupport_introspection_cpp::SetObs_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asv_interfaces, srv, SetObs_Request)() {
  return &::asv_interfaces::srv::rosidl_typesupport_introspection_cpp::SetObs_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "asv_interfaces/srv/detail/set_obs__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace asv_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetObs_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asv_interfaces::srv::SetObs_Response(_init);
}

void SetObs_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asv_interfaces::srv::SetObs_Response *>(message_memory);
  typed_message->~SetObs_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetObs_Response_message_member_array[1] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asv_interfaces::srv::SetObs_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetObs_Response_message_members = {
  "asv_interfaces::srv",  // message namespace
  "SetObs_Response",  // message name
  1,  // number of fields
  sizeof(asv_interfaces::srv::SetObs_Response),
  SetObs_Response_message_member_array,  // message members
  SetObs_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetObs_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetObs_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetObs_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace asv_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<asv_interfaces::srv::SetObs_Response>()
{
  return &::asv_interfaces::srv::rosidl_typesupport_introspection_cpp::SetObs_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asv_interfaces, srv, SetObs_Response)() {
  return &::asv_interfaces::srv::rosidl_typesupport_introspection_cpp::SetObs_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "asv_interfaces/srv/detail/set_obs__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace asv_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SetObs_service_members = {
  "asv_interfaces::srv",  // service namespace
  "SetObs",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<asv_interfaces::srv::SetObs>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SetObs_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetObs_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace asv_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<asv_interfaces::srv::SetObs>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::asv_interfaces::srv::rosidl_typesupport_introspection_cpp::SetObs_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::asv_interfaces::srv::SetObs_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::asv_interfaces::srv::SetObs_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asv_interfaces, srv, SetObs)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<asv_interfaces::srv::SetObs>();
}

#ifdef __cplusplus
}
#endif
