cmake_minimum_required(VERSION 3.5)
project(originman_action_imitation)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development)

# 设置包含目录
include_directories(
  include
  ${Python3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(action_imitation_node
  src/main.cpp
  src/order_interpreter.cpp
  src/action_imitation.cpp
)

# 链接库
target_link_libraries(action_imitation_node
  ${rclcpp_LIBRARIES}
  ${ai_msgs_LIBRARIES}
  ${Python3_LIBRARIES}
)

# 添加依赖
ament_target_dependencies(action_imitation_node
  rclcpp
  ai_msgs
)

# 安装目标文件
install(
  TARGETS action_imitation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装目录（可选，如果有其他资源文件）
install(
  DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch目录（如果存在）
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 打包ament
ament_package()