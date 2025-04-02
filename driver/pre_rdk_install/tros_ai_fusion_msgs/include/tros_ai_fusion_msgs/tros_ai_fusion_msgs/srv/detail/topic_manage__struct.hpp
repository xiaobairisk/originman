// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice

#ifndef TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__STRUCT_HPP_
#define TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Request __attribute__((deprecated))
#else
# define DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Request __declspec(deprecated)
#endif

namespace tros_ai_fusion_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TopicManage_Request_
{
  using Type = TopicManage_Request_<ContainerAllocator>;

  explicit TopicManage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action = "";
    }
  }

  explicit TopicManage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : action(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action = "";
    }
  }

  // field types and members
  using _action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_type action;
  using _topics_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _topics_type topics;

  // setters for named parameter idiom
  Type & set__action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__topics(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->topics = _arg;
    return *this;
  }

  // constant declarations
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> ADD;
  // guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> DELETE;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> GET;

  // pointer types
  using RawPtr =
    tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Request
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Request
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TopicManage_Request_ & other) const
  {
    if (this->action != other.action) {
      return false;
    }
    if (this->topics != other.topics) {
      return false;
    }
    return true;
  }
  bool operator!=(const TopicManage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TopicManage_Request_

// alias to use template instance with default allocator
using TopicManage_Request =
  tros_ai_fusion_msgs::srv::TopicManage_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
TopicManage_Request_<ContainerAllocator>::ADD = "add";
// guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
TopicManage_Request_<ContainerAllocator>::DELETE = "delete";
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
TopicManage_Request_<ContainerAllocator>::GET = "get";

}  // namespace srv

}  // namespace tros_ai_fusion_msgs


#ifndef _WIN32
# define DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Response __attribute__((deprecated))
#else
# define DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Response __declspec(deprecated)
#endif

namespace tros_ai_fusion_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TopicManage_Response_
{
  using Type = TopicManage_Response_<ContainerAllocator>;

  explicit TopicManage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit TopicManage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;
  using _topics_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _topics_type topics;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__topics(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->topics = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Response
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tros_ai_fusion_msgs__srv__TopicManage_Response
    std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TopicManage_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->topics != other.topics) {
      return false;
    }
    return true;
  }
  bool operator!=(const TopicManage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TopicManage_Response_

// alias to use template instance with default allocator
using TopicManage_Response =
  tros_ai_fusion_msgs::srv::TopicManage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tros_ai_fusion_msgs

namespace tros_ai_fusion_msgs
{

namespace srv
{

struct TopicManage
{
  using Request = tros_ai_fusion_msgs::srv::TopicManage_Request;
  using Response = tros_ai_fusion_msgs::srv::TopicManage_Response;
};

}  // namespace srv

}  // namespace tros_ai_fusion_msgs

#endif  // TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__STRUCT_HPP_
