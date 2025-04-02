// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice

#ifndef TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__STRUCT_H_
#define TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ADD'.
static const char * const tros_ai_fusion_msgs__srv__TopicManage_Request__ADD = "add";

/// Constant 'DELETE'.
static const char * const tros_ai_fusion_msgs__srv__TopicManage_Request__DELETE = "delete";

/// Constant 'GET'.
static const char * const tros_ai_fusion_msgs__srv__TopicManage_Request__GET = "get";

// Include directives for member types
// Member 'action'
// Member 'topics'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TopicManage in the package tros_ai_fusion_msgs.
typedef struct tros_ai_fusion_msgs__srv__TopicManage_Request
{
  rosidl_runtime_c__String action;
  rosidl_runtime_c__String__Sequence topics;
} tros_ai_fusion_msgs__srv__TopicManage_Request;

// Struct for a sequence of tros_ai_fusion_msgs__srv__TopicManage_Request.
typedef struct tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence
{
  tros_ai_fusion_msgs__srv__TopicManage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'topics'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TopicManage in the package tros_ai_fusion_msgs.
typedef struct tros_ai_fusion_msgs__srv__TopicManage_Response
{
  bool result;
  rosidl_runtime_c__String__Sequence topics;
} tros_ai_fusion_msgs__srv__TopicManage_Response;

// Struct for a sequence of tros_ai_fusion_msgs__srv__TopicManage_Response.
typedef struct tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence
{
  tros_ai_fusion_msgs__srv__TopicManage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__STRUCT_H_
