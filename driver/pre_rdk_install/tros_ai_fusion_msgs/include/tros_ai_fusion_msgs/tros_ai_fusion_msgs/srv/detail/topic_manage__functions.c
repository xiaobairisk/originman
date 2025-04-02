// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice
#include "tros_ai_fusion_msgs/srv/detail/topic_manage__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `action`
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"

bool
tros_ai_fusion_msgs__srv__TopicManage_Request__init(tros_ai_fusion_msgs__srv__TopicManage_Request * msg)
{
  if (!msg) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__init(&msg->action)) {
    tros_ai_fusion_msgs__srv__TopicManage_Request__fini(msg);
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__init(&msg->topics, 0)) {
    tros_ai_fusion_msgs__srv__TopicManage_Request__fini(msg);
    return false;
  }
  return true;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Request__fini(tros_ai_fusion_msgs__srv__TopicManage_Request * msg)
{
  if (!msg) {
    return;
  }
  // action
  rosidl_runtime_c__String__fini(&msg->action);
  // topics
  rosidl_runtime_c__String__Sequence__fini(&msg->topics);
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Request__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Request * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action), &(rhs->action)))
  {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->topics), &(rhs->topics)))
  {
    return false;
  }
  return true;
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Request__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Request * input,
  tros_ai_fusion_msgs__srv__TopicManage_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__copy(
      &(input->action), &(output->action)))
  {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->topics), &(output->topics)))
  {
    return false;
  }
  return true;
}

tros_ai_fusion_msgs__srv__TopicManage_Request *
tros_ai_fusion_msgs__srv__TopicManage_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tros_ai_fusion_msgs__srv__TopicManage_Request * msg = (tros_ai_fusion_msgs__srv__TopicManage_Request *)allocator.allocate(sizeof(tros_ai_fusion_msgs__srv__TopicManage_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tros_ai_fusion_msgs__srv__TopicManage_Request));
  bool success = tros_ai_fusion_msgs__srv__TopicManage_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Request__destroy(tros_ai_fusion_msgs__srv__TopicManage_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tros_ai_fusion_msgs__srv__TopicManage_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__init(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tros_ai_fusion_msgs__srv__TopicManage_Request * data = NULL;

  if (size) {
    data = (tros_ai_fusion_msgs__srv__TopicManage_Request *)allocator.zero_allocate(size, sizeof(tros_ai_fusion_msgs__srv__TopicManage_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tros_ai_fusion_msgs__srv__TopicManage_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tros_ai_fusion_msgs__srv__TopicManage_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__fini(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tros_ai_fusion_msgs__srv__TopicManage_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence *
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array = (tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence *)allocator.allocate(sizeof(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__destroy(tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tros_ai_fusion_msgs__srv__TopicManage_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * input,
  tros_ai_fusion_msgs__srv__TopicManage_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tros_ai_fusion_msgs__srv__TopicManage_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tros_ai_fusion_msgs__srv__TopicManage_Request * data =
      (tros_ai_fusion_msgs__srv__TopicManage_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tros_ai_fusion_msgs__srv__TopicManage_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tros_ai_fusion_msgs__srv__TopicManage_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tros_ai_fusion_msgs__srv__TopicManage_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `topics`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
tros_ai_fusion_msgs__srv__TopicManage_Response__init(tros_ai_fusion_msgs__srv__TopicManage_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  // topics
  if (!rosidl_runtime_c__String__Sequence__init(&msg->topics, 0)) {
    tros_ai_fusion_msgs__srv__TopicManage_Response__fini(msg);
    return false;
  }
  return true;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Response__fini(tros_ai_fusion_msgs__srv__TopicManage_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
  // topics
  rosidl_runtime_c__String__Sequence__fini(&msg->topics);
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Response__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Response * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->topics), &(rhs->topics)))
  {
    return false;
  }
  return true;
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Response__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Response * input,
  tros_ai_fusion_msgs__srv__TopicManage_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  // topics
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->topics), &(output->topics)))
  {
    return false;
  }
  return true;
}

tros_ai_fusion_msgs__srv__TopicManage_Response *
tros_ai_fusion_msgs__srv__TopicManage_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tros_ai_fusion_msgs__srv__TopicManage_Response * msg = (tros_ai_fusion_msgs__srv__TopicManage_Response *)allocator.allocate(sizeof(tros_ai_fusion_msgs__srv__TopicManage_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tros_ai_fusion_msgs__srv__TopicManage_Response));
  bool success = tros_ai_fusion_msgs__srv__TopicManage_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Response__destroy(tros_ai_fusion_msgs__srv__TopicManage_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tros_ai_fusion_msgs__srv__TopicManage_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__init(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tros_ai_fusion_msgs__srv__TopicManage_Response * data = NULL;

  if (size) {
    data = (tros_ai_fusion_msgs__srv__TopicManage_Response *)allocator.zero_allocate(size, sizeof(tros_ai_fusion_msgs__srv__TopicManage_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tros_ai_fusion_msgs__srv__TopicManage_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tros_ai_fusion_msgs__srv__TopicManage_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__fini(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tros_ai_fusion_msgs__srv__TopicManage_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence *
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array = (tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence *)allocator.allocate(sizeof(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__destroy(tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__are_equal(const tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * lhs, const tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tros_ai_fusion_msgs__srv__TopicManage_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence__copy(
  const tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * input,
  tros_ai_fusion_msgs__srv__TopicManage_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tros_ai_fusion_msgs__srv__TopicManage_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tros_ai_fusion_msgs__srv__TopicManage_Response * data =
      (tros_ai_fusion_msgs__srv__TopicManage_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tros_ai_fusion_msgs__srv__TopicManage_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tros_ai_fusion_msgs__srv__TopicManage_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tros_ai_fusion_msgs__srv__TopicManage_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
