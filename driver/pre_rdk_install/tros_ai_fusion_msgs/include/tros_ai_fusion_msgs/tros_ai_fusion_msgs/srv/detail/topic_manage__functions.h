// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice

#ifndef TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__FUNCTIONS_H_
#define TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tros_ai_fusion_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "tros_ai_fusion_msgs/srv/detail/topic_manage__struct.h"

/// Initialize srv/TopicManage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tros_ai_fusion_msgs__srv__TopicManage_Request
 * )) before or use
 * tros_ai_fusion_msgs__srv__TopicManage_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Request__init(tros_ai_fusion_msgs__srv__TopicManage_Request * msg);

/// Finalize srv/TopicManage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Request__fini(tros_ai_fusion_msgs__srv__TopicManage_Request * msg);

/// Create srv/TopicManage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tros_ai_fusion_msgs__srv__TopicManage_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
tros_ai_fusion_msgs__srv__TopicManage_Request *
tros_ai_fusion_msgs__srv__TopicManage_Request__create();

/// Destroy srv/TopicManage message.
/**
 * It calls
 * tros_ai_fusion_msgs__srv__TopicManage_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Request__destroy(tros_ai_fusion_msgs__srv__TopicManage_Request * msg);

/// Check for srv/TopicManage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Request__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Request * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Request * rhs);

/// Copy a srv/TopicManage message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Request__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Request * input,
  tros_ai_fusion_msgs__srv__TopicManage_Request * output);

/// Initialize array of srv/TopicManage messages.
/**
 * It allocates the memory for the number of elements and calls
 * tros_ai_fusion_msgs__srv__TopicManage_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__init(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array, size_t size);

/// Finalize array of srv/TopicManage messages.
/**
 * It calls
 * tros_ai_fusion_msgs__srv__TopicManage_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__fini(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array);

/// Create array of srv/TopicManage messages.
/**
 * It allocates the memory for the array and calls
 * tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence *
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__create(size_t size);

/// Destroy array of srv/TopicManage messages.
/**
 * It calls
 * tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__destroy(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array);

/// Check for srv/TopicManage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * rhs);

/// Copy an array of srv/TopicManage messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * input,
  tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * output);

/// Initialize srv/TopicManage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tros_ai_fusion_msgs__srv__TopicManage_Response
 * )) before or use
 * tros_ai_fusion_msgs__srv__TopicManage_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Response__init(tros_ai_fusion_msgs__srv__TopicManage_Response * msg);

/// Finalize srv/TopicManage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Response__fini(tros_ai_fusion_msgs__srv__TopicManage_Response * msg);

/// Create srv/TopicManage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tros_ai_fusion_msgs__srv__TopicManage_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
tros_ai_fusion_msgs__srv__TopicManage_Response *
tros_ai_fusion_msgs__srv__TopicManage_Response__create();

/// Destroy srv/TopicManage message.
/**
 * It calls
 * tros_ai_fusion_msgs__srv__TopicManage_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Response__destroy(tros_ai_fusion_msgs__srv__TopicManage_Response * msg);

/// Check for srv/TopicManage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Response__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Response * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Response * rhs);

/// Copy a srv/TopicManage message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Response__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Response * input,
  tros_ai_fusion_msgs__srv__TopicManage_Response * output);

/// Initialize array of srv/TopicManage messages.
/**
 * It allocates the memory for the number of elements and calls
 * tros_ai_fusion_msgs__srv__TopicManage_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__init(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array, size_t size);

/// Finalize array of srv/TopicManage messages.
/**
 * It calls
 * tros_ai_fusion_msgs__srv__TopicManage_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__fini(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array);

/// Create array of srv/TopicManage messages.
/**
 * It allocates the memory for the array and calls
 * tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence *
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__create(size_t size);

/// Destroy array of srv/TopicManage messages.
/**
 * It calls
 * tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
void
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__destroy(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array);

/// Check for srv/TopicManage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * rhs);

/// Copy an array of srv/TopicManage messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tros_ai_fusion_msgs
bool
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * input,
  tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__FUNCTIONS_H_
