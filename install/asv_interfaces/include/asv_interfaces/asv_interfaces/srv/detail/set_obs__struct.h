// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asv_interfaces:srv/SetObs.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__SRV__DETAIL__SET_OBS__STRUCT_H_
#define ASV_INTERFACES__SRV__DETAIL__SET_OBS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ESO_BEJARANO'.
/**
  * basic ESO_MODE
 */
enum
{
  asv_interfaces__srv__SetObs_Request__ESO_BEJARANO = 1
};

/// Constant 'ESO_LIU'.
enum
{
  asv_interfaces__srv__SetObs_Request__ESO_LIU = 2
};

/// Constant 'ESO_ZONO'.
enum
{
  asv_interfaces__srv__SetObs_Request__ESO_ZONO = 3
};

/// Struct defined in srv/SetObs in the package asv_interfaces.
typedef struct asv_interfaces__srv__SetObs_Request
{
  /// filled by ESO_MODE enum value
  uint8_t eso_mode;
} asv_interfaces__srv__SetObs_Request;

// Struct for a sequence of asv_interfaces__srv__SetObs_Request.
typedef struct asv_interfaces__srv__SetObs_Request__Sequence
{
  asv_interfaces__srv__SetObs_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__srv__SetObs_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetObs in the package asv_interfaces.
typedef struct asv_interfaces__srv__SetObs_Response
{
  /// Mode correctly and SET_ESO are sent
  bool success;
} asv_interfaces__srv__SetObs_Response;

// Struct for a sequence of asv_interfaces__srv__SetObs_Response.
typedef struct asv_interfaces__srv__SetObs_Response__Sequence
{
  asv_interfaces__srv__SetObs_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asv_interfaces__srv__SetObs_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASV_INTERFACES__SRV__DETAIL__SET_OBS__STRUCT_H_
