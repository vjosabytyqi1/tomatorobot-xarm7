// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xarm_msgs:srv/TrajCtrl.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__TRAJ_CTRL__STRUCT_H_
#define XARM_MSGS__SRV__DETAIL__TRAJ_CTRL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filename'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TrajCtrl in the package xarm_msgs.
typedef struct xarm_msgs__srv__TrajCtrl_Request
{
  rosidl_runtime_c__String filename;
  float timeout;
} xarm_msgs__srv__TrajCtrl_Request;

// Struct for a sequence of xarm_msgs__srv__TrajCtrl_Request.
typedef struct xarm_msgs__srv__TrajCtrl_Request__Sequence
{
  xarm_msgs__srv__TrajCtrl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__srv__TrajCtrl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TrajCtrl in the package xarm_msgs.
typedef struct xarm_msgs__srv__TrajCtrl_Response
{
  int16_t ret;
  rosidl_runtime_c__String message;
} xarm_msgs__srv__TrajCtrl_Response;

// Struct for a sequence of xarm_msgs__srv__TrajCtrl_Response.
typedef struct xarm_msgs__srv__TrajCtrl_Response__Sequence
{
  xarm_msgs__srv__TrajCtrl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__srv__TrajCtrl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__SRV__DETAIL__TRAJ_CTRL__STRUCT_H_
