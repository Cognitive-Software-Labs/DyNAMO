// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from explore_lite_msgs:msg/ExploreStatus.idl
// generated code does not contain a copyright notice
#include "explore_lite_msgs/msg/detail/explore_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "explore_lite_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "explore_lite_msgs/msg/detail/explore_status__struct.h"
#include "explore_lite_msgs/msg/detail/explore_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // status
#include "rosidl_runtime_c/string_functions.h"  // status

// forward declare type support functions


using _ExploreStatus__ros_msg_type = explore_lite_msgs__msg__ExploreStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
bool cdr_serialize_explore_lite_msgs__msg__ExploreStatus(
  const explore_lite_msgs__msg__ExploreStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: status
  {
    const rosidl_runtime_c__String * str = &ros_message->status;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
bool cdr_deserialize_explore_lite_msgs__msg__ExploreStatus(
  eprosima::fastcdr::Cdr & cdr,
  explore_lite_msgs__msg__ExploreStatus * ros_message)
{
  // Field name: status
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->status.data) {
      rosidl_runtime_c__String__init(&ros_message->status);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->status,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'status'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
size_t get_serialized_size_explore_lite_msgs__msg__ExploreStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ExploreStatus__ros_msg_type * ros_message = static_cast<const _ExploreStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->status.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
size_t max_serialized_size_explore_lite_msgs__msg__ExploreStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: status
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = explore_lite_msgs__msg__ExploreStatus;
    is_plain =
      (
      offsetof(DataType, status) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
bool cdr_serialize_key_explore_lite_msgs__msg__ExploreStatus(
  const explore_lite_msgs__msg__ExploreStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: status
  {
    const rosidl_runtime_c__String * str = &ros_message->status;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
size_t get_serialized_size_key_explore_lite_msgs__msg__ExploreStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ExploreStatus__ros_msg_type * ros_message = static_cast<const _ExploreStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->status.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_explore_lite_msgs
size_t max_serialized_size_key_explore_lite_msgs__msg__ExploreStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: status
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = explore_lite_msgs__msg__ExploreStatus;
    is_plain =
      (
      offsetof(DataType, status) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ExploreStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const explore_lite_msgs__msg__ExploreStatus * ros_message = static_cast<const explore_lite_msgs__msg__ExploreStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_explore_lite_msgs__msg__ExploreStatus(ros_message, cdr);
}

static bool _ExploreStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  explore_lite_msgs__msg__ExploreStatus * ros_message = static_cast<explore_lite_msgs__msg__ExploreStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_explore_lite_msgs__msg__ExploreStatus(cdr, ros_message);
}

static uint32_t _ExploreStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_explore_lite_msgs__msg__ExploreStatus(
      untyped_ros_message, 0));
}

static size_t _ExploreStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_explore_lite_msgs__msg__ExploreStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ExploreStatus = {
  "explore_lite_msgs::msg",
  "ExploreStatus",
  _ExploreStatus__cdr_serialize,
  _ExploreStatus__cdr_deserialize,
  _ExploreStatus__get_serialized_size,
  _ExploreStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ExploreStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ExploreStatus,
  get_message_typesupport_handle_function,
  &explore_lite_msgs__msg__ExploreStatus__get_type_hash,
  &explore_lite_msgs__msg__ExploreStatus__get_type_description,
  &explore_lite_msgs__msg__ExploreStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, explore_lite_msgs, msg, ExploreStatus)() {
  return &_ExploreStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
