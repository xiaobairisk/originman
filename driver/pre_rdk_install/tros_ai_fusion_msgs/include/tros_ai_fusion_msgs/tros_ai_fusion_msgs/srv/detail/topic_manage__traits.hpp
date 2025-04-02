// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice

#ifndef TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__TRAITS_HPP_
#define TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tros_ai_fusion_msgs/srv/detail/topic_manage__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tros_ai_fusion_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const TopicManage_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: topics
  {
    if (msg.topics.size() == 0) {
      out << "topics: []";
    } else {
      out << "topics: [";
      size_t pending_items = msg.topics.size();
      for (auto item : msg.topics) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TopicManage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: topics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topics.size() == 0) {
      out << "topics: []\n";
    } else {
      out << "topics:\n";
      for (auto item : msg.topics) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TopicManage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tros_ai_fusion_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tros_ai_fusion_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tros_ai_fusion_msgs::srv::TopicManage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tros_ai_fusion_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tros_ai_fusion_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tros_ai_fusion_msgs::srv::TopicManage_Request & msg)
{
  return tros_ai_fusion_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tros_ai_fusion_msgs::srv::TopicManage_Request>()
{
  return "tros_ai_fusion_msgs::srv::TopicManage_Request";
}

template<>
inline const char * name<tros_ai_fusion_msgs::srv::TopicManage_Request>()
{
  return "tros_ai_fusion_msgs/srv/TopicManage_Request";
}

template<>
struct has_fixed_size<tros_ai_fusion_msgs::srv::TopicManage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tros_ai_fusion_msgs::srv::TopicManage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tros_ai_fusion_msgs::srv::TopicManage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace tros_ai_fusion_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const TopicManage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << ", ";
  }

  // member: topics
  {
    if (msg.topics.size() == 0) {
      out << "topics: []";
    } else {
      out << "topics: [";
      size_t pending_items = msg.topics.size();
      for (auto item : msg.topics) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TopicManage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }

  // member: topics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topics.size() == 0) {
      out << "topics: []\n";
    } else {
      out << "topics:\n";
      for (auto item : msg.topics) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TopicManage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tros_ai_fusion_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tros_ai_fusion_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tros_ai_fusion_msgs::srv::TopicManage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tros_ai_fusion_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tros_ai_fusion_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const tros_ai_fusion_msgs::srv::TopicManage_Response & msg)
{
  return tros_ai_fusion_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tros_ai_fusion_msgs::srv::TopicManage_Response>()
{
  return "tros_ai_fusion_msgs::srv::TopicManage_Response";
}

template<>
inline const char * name<tros_ai_fusion_msgs::srv::TopicManage_Response>()
{
  return "tros_ai_fusion_msgs/srv/TopicManage_Response";
}

template<>
struct has_fixed_size<tros_ai_fusion_msgs::srv::TopicManage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tros_ai_fusion_msgs::srv::TopicManage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tros_ai_fusion_msgs::srv::TopicManage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tros_ai_fusion_msgs::srv::TopicManage>()
{
  return "tros_ai_fusion_msgs::srv::TopicManage";
}

template<>
inline const char * name<tros_ai_fusion_msgs::srv::TopicManage>()
{
  return "tros_ai_fusion_msgs/srv/TopicManage";
}

template<>
struct has_fixed_size<tros_ai_fusion_msgs::srv::TopicManage>
  : std::integral_constant<
    bool,
    has_fixed_size<tros_ai_fusion_msgs::srv::TopicManage_Request>::value &&
    has_fixed_size<tros_ai_fusion_msgs::srv::TopicManage_Response>::value
  >
{
};

template<>
struct has_bounded_size<tros_ai_fusion_msgs::srv::TopicManage>
  : std::integral_constant<
    bool,
    has_bounded_size<tros_ai_fusion_msgs::srv::TopicManage_Request>::value &&
    has_bounded_size<tros_ai_fusion_msgs::srv::TopicManage_Response>::value
  >
{
};

template<>
struct is_service<tros_ai_fusion_msgs::srv::TopicManage>
  : std::true_type
{
};

template<>
struct is_service_request<tros_ai_fusion_msgs::srv::TopicManage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tros_ai_fusion_msgs::srv::TopicManage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__TRAITS_HPP_
