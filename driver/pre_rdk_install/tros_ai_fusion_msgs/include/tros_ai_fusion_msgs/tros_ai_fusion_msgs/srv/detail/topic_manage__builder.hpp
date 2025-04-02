// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tros_ai_fusion_msgs:srv/TopicManage.idl
// generated code does not contain a copyright notice

#ifndef TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__BUILDER_HPP_
#define TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tros_ai_fusion_msgs/srv/detail/topic_manage__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tros_ai_fusion_msgs
{

namespace srv
{

namespace builder
{

class Init_TopicManage_Request_topics
{
public:
  explicit Init_TopicManage_Request_topics(::tros_ai_fusion_msgs::srv::TopicManage_Request & msg)
  : msg_(msg)
  {}
  ::tros_ai_fusion_msgs::srv::TopicManage_Request topics(::tros_ai_fusion_msgs::srv::TopicManage_Request::_topics_type arg)
  {
    msg_.topics = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tros_ai_fusion_msgs::srv::TopicManage_Request msg_;
};

class Init_TopicManage_Request_action
{
public:
  Init_TopicManage_Request_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TopicManage_Request_topics action(::tros_ai_fusion_msgs::srv::TopicManage_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_TopicManage_Request_topics(msg_);
  }

private:
  ::tros_ai_fusion_msgs::srv::TopicManage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tros_ai_fusion_msgs::srv::TopicManage_Request>()
{
  return tros_ai_fusion_msgs::srv::builder::Init_TopicManage_Request_action();
}

}  // namespace tros_ai_fusion_msgs


namespace tros_ai_fusion_msgs
{

namespace srv
{

namespace builder
{

class Init_TopicManage_Response_topics
{
public:
  explicit Init_TopicManage_Response_topics(::tros_ai_fusion_msgs::srv::TopicManage_Response & msg)
  : msg_(msg)
  {}
  ::tros_ai_fusion_msgs::srv::TopicManage_Response topics(::tros_ai_fusion_msgs::srv::TopicManage_Response::_topics_type arg)
  {
    msg_.topics = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tros_ai_fusion_msgs::srv::TopicManage_Response msg_;
};

class Init_TopicManage_Response_result
{
public:
  Init_TopicManage_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TopicManage_Response_topics result(::tros_ai_fusion_msgs::srv::TopicManage_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_TopicManage_Response_topics(msg_);
  }

private:
  ::tros_ai_fusion_msgs::srv::TopicManage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tros_ai_fusion_msgs::srv::TopicManage_Response>()
{
  return tros_ai_fusion_msgs::srv::builder::Init_TopicManage_Response_result();
}

}  // namespace tros_ai_fusion_msgs

#endif  // TROS_AI_FUSION_MSGS__SRV__DETAIL__TOPIC_MANAGE__BUILDER_HPP_
