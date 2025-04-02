// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tros_ai_fusion_msgs/srv/detail/topic_manage__rosidl_typesupport_introspection_c.h"
#include "tros_ai_fusion_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tros_ai_fusion_msgs/srv/detail/topic_manage__functions.h"
#include "tros_ai_fusion_msgs/srv/detail/topic_manage__struct.h"


// Include directives for member types
// Member `action`
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tros_ai_fusion_msgs__srv__TopicManage_Request__init(message_memory);
}

void tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_fini_function(void * message_memory)
{
  tros_ai_fusion_msgs__srv__TopicManage_Request__fini(message_memory);
}

size_t tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__size_function__TopicManage_Request__topics(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__get_const_function__TopicManage_Request__topics(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__get_function__TopicManage_Request__topics(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__fetch_function__TopicManage_Request__topics(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__get_const_function__TopicManage_Request__topics(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__assign_function__TopicManage_Request__topics(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__get_function__TopicManage_Request__topics(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__resize_function__TopicManage_Request__topics(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_member_array[2] = {
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tros_ai_fusion_msgs__srv__TopicManage_Request, action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "topics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tros_ai_fusion_msgs__srv__TopicManage_Request, topics),  // bytes offset in struct
    NULL,  // default value
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__size_function__TopicManage_Request__topics,  // size() function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__get_const_function__TopicManage_Request__topics,  // get_const(index) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__get_function__TopicManage_Request__topics,  // get(index) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__fetch_function__TopicManage_Request__topics,  // fetch(index, &value) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__assign_function__TopicManage_Request__topics,  // assign(index, value) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__resize_function__TopicManage_Request__topics  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_members = {
  "tros_ai_fusion_msgs__srv",  // message namespace
  "TopicManage_Request",  // message name
  2,  // number of fields
  sizeof(tros_ai_fusion_msgs__srv__TopicManage_Request),
  tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_member_array,  // message members
  tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_type_support_handle = {
  0,
  &tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tros_ai_fusion_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage_Request)() {
  if (!tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_type_support_handle.typesupport_identifier) {
    tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tros_ai_fusion_msgs__srv__TopicManage_Request__rosidl_typesupport_introspection_c__TopicManage_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "tros_ai_fusion_msgs/srv/detail/topic_manage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "tros_ai_fusion_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "tros_ai_fusion_msgs/srv/detail/topic_manage__functions.h"
// already included above
// #include "tros_ai_fusion_msgs/srv/detail/topic_manage__struct.h"


// Include directives for member types
// Member `topics`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tros_ai_fusion_msgs__srv__TopicManage_Response__init(message_memory);
}

void tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_fini_function(void * message_memory)
{
  tros_ai_fusion_msgs__srv__TopicManage_Response__fini(message_memory);
}

size_t tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__size_function__TopicManage_Response__topics(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__get_const_function__TopicManage_Response__topics(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__get_function__TopicManage_Response__topics(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__fetch_function__TopicManage_Response__topics(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__get_const_function__TopicManage_Response__topics(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__assign_function__TopicManage_Response__topics(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__get_function__TopicManage_Response__topics(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__resize_function__TopicManage_Response__topics(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_member_array[2] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tros_ai_fusion_msgs__srv__TopicManage_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "topics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tros_ai_fusion_msgs__srv__TopicManage_Response, topics),  // bytes offset in struct
    NULL,  // default value
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__size_function__TopicManage_Response__topics,  // size() function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__get_const_function__TopicManage_Response__topics,  // get_const(index) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__get_function__TopicManage_Response__topics,  // get(index) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__fetch_function__TopicManage_Response__topics,  // fetch(index, &value) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__assign_function__TopicManage_Response__topics,  // assign(index, value) function pointer
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__resize_function__TopicManage_Response__topics  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_members = {
  "tros_ai_fusion_msgs__srv",  // message namespace
  "TopicManage_Response",  // message name
  2,  // number of fields
  sizeof(tros_ai_fusion_msgs__srv__TopicManage_Response),
  tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_member_array,  // message members
  tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_type_support_handle = {
  0,
  &tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tros_ai_fusion_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage_Response)() {
  if (!tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_type_support_handle.typesupport_identifier) {
    tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tros_ai_fusion_msgs__srv__TopicManage_Response__rosidl_typesupport_introspection_c__TopicManage_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "tros_ai_fusion_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "tros_ai_fusion_msgs/srv/detail/topic_manage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_members = {
  "tros_ai_fusion_msgs__srv",  // service namespace
  "TopicManage",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_Request_message_type_support_handle,
  NULL  // response message
  // tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_Response_message_type_support_handle
};

static rosidl_service_type_support_t tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_type_support_handle = {
  0,
  &tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tros_ai_fusion_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage)() {
  if (!tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_type_support_handle.typesupport_identifier) {
    tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tros_ai_fusion_msgs, srv, TopicManage_Response)()->data;
  }

  return &tros_ai_fusion_msgs__srv__detail__topic_manage__rosidl_typesupport_introspection_c__TopicManage_service_type_support_handle;
}
