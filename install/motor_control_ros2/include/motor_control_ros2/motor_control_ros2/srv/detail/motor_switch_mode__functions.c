// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_control_ros2:srv/MotorSwitchMode.idl
// generated code does not contain a copyright notice
#include "motor_control_ros2/srv/detail/motor_switch_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `motor_name`
#include "rosidl_runtime_c/string_functions.h"

bool
motor_control_ros2__srv__MotorSwitchMode_Request__init(motor_control_ros2__srv__MotorSwitchMode_Request * msg)
{
  if (!msg) {
    return false;
  }
  // motor_name
  if (!rosidl_runtime_c__String__init(&msg->motor_name)) {
    motor_control_ros2__srv__MotorSwitchMode_Request__fini(msg);
    return false;
  }
  // mode
  return true;
}

void
motor_control_ros2__srv__MotorSwitchMode_Request__fini(motor_control_ros2__srv__MotorSwitchMode_Request * msg)
{
  if (!msg) {
    return;
  }
  // motor_name
  rosidl_runtime_c__String__fini(&msg->motor_name);
  // mode
}

bool
motor_control_ros2__srv__MotorSwitchMode_Request__are_equal(const motor_control_ros2__srv__MotorSwitchMode_Request * lhs, const motor_control_ros2__srv__MotorSwitchMode_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor_name), &(rhs->motor_name)))
  {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
motor_control_ros2__srv__MotorSwitchMode_Request__copy(
  const motor_control_ros2__srv__MotorSwitchMode_Request * input,
  motor_control_ros2__srv__MotorSwitchMode_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_name
  if (!rosidl_runtime_c__String__copy(
      &(input->motor_name), &(output->motor_name)))
  {
    return false;
  }
  // mode
  output->mode = input->mode;
  return true;
}

motor_control_ros2__srv__MotorSwitchMode_Request *
motor_control_ros2__srv__MotorSwitchMode_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__srv__MotorSwitchMode_Request * msg = (motor_control_ros2__srv__MotorSwitchMode_Request *)allocator.allocate(sizeof(motor_control_ros2__srv__MotorSwitchMode_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_control_ros2__srv__MotorSwitchMode_Request));
  bool success = motor_control_ros2__srv__MotorSwitchMode_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_control_ros2__srv__MotorSwitchMode_Request__destroy(motor_control_ros2__srv__MotorSwitchMode_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_control_ros2__srv__MotorSwitchMode_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__init(motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__srv__MotorSwitchMode_Request * data = NULL;

  if (size) {
    data = (motor_control_ros2__srv__MotorSwitchMode_Request *)allocator.zero_allocate(size, sizeof(motor_control_ros2__srv__MotorSwitchMode_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_control_ros2__srv__MotorSwitchMode_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_control_ros2__srv__MotorSwitchMode_Request__fini(&data[i - 1]);
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
motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__fini(motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * array)
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
      motor_control_ros2__srv__MotorSwitchMode_Request__fini(&array->data[i]);
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

motor_control_ros2__srv__MotorSwitchMode_Request__Sequence *
motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * array = (motor_control_ros2__srv__MotorSwitchMode_Request__Sequence *)allocator.allocate(sizeof(motor_control_ros2__srv__MotorSwitchMode_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__destroy(motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__are_equal(const motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * lhs, const motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_control_ros2__srv__MotorSwitchMode_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_control_ros2__srv__MotorSwitchMode_Request__Sequence__copy(
  const motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * input,
  motor_control_ros2__srv__MotorSwitchMode_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_control_ros2__srv__MotorSwitchMode_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_control_ros2__srv__MotorSwitchMode_Request * data =
      (motor_control_ros2__srv__MotorSwitchMode_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_control_ros2__srv__MotorSwitchMode_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_control_ros2__srv__MotorSwitchMode_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_control_ros2__srv__MotorSwitchMode_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
motor_control_ros2__srv__MotorSwitchMode_Response__init(motor_control_ros2__srv__MotorSwitchMode_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    motor_control_ros2__srv__MotorSwitchMode_Response__fini(msg);
    return false;
  }
  return true;
}

void
motor_control_ros2__srv__MotorSwitchMode_Response__fini(motor_control_ros2__srv__MotorSwitchMode_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
motor_control_ros2__srv__MotorSwitchMode_Response__are_equal(const motor_control_ros2__srv__MotorSwitchMode_Response * lhs, const motor_control_ros2__srv__MotorSwitchMode_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
motor_control_ros2__srv__MotorSwitchMode_Response__copy(
  const motor_control_ros2__srv__MotorSwitchMode_Response * input,
  motor_control_ros2__srv__MotorSwitchMode_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

motor_control_ros2__srv__MotorSwitchMode_Response *
motor_control_ros2__srv__MotorSwitchMode_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__srv__MotorSwitchMode_Response * msg = (motor_control_ros2__srv__MotorSwitchMode_Response *)allocator.allocate(sizeof(motor_control_ros2__srv__MotorSwitchMode_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_control_ros2__srv__MotorSwitchMode_Response));
  bool success = motor_control_ros2__srv__MotorSwitchMode_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_control_ros2__srv__MotorSwitchMode_Response__destroy(motor_control_ros2__srv__MotorSwitchMode_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_control_ros2__srv__MotorSwitchMode_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__init(motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__srv__MotorSwitchMode_Response * data = NULL;

  if (size) {
    data = (motor_control_ros2__srv__MotorSwitchMode_Response *)allocator.zero_allocate(size, sizeof(motor_control_ros2__srv__MotorSwitchMode_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_control_ros2__srv__MotorSwitchMode_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_control_ros2__srv__MotorSwitchMode_Response__fini(&data[i - 1]);
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
motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__fini(motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * array)
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
      motor_control_ros2__srv__MotorSwitchMode_Response__fini(&array->data[i]);
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

motor_control_ros2__srv__MotorSwitchMode_Response__Sequence *
motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * array = (motor_control_ros2__srv__MotorSwitchMode_Response__Sequence *)allocator.allocate(sizeof(motor_control_ros2__srv__MotorSwitchMode_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__destroy(motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__are_equal(const motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * lhs, const motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_control_ros2__srv__MotorSwitchMode_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_control_ros2__srv__MotorSwitchMode_Response__Sequence__copy(
  const motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * input,
  motor_control_ros2__srv__MotorSwitchMode_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_control_ros2__srv__MotorSwitchMode_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_control_ros2__srv__MotorSwitchMode_Response * data =
      (motor_control_ros2__srv__MotorSwitchMode_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_control_ros2__srv__MotorSwitchMode_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_control_ros2__srv__MotorSwitchMode_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_control_ros2__srv__MotorSwitchMode_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
