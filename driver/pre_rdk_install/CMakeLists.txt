cmake_minimum_required(VERSION 3.8)
project(pre_rdk_install)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ament_cmake 包
find_package(ament_cmake REQUIRED)

# 获取当前包的上级目录（即 /userdata/dev_ws/install）
get_filename_component(install_parent_dir ${CMAKE_INSTALL_PREFIX} DIRECTORY)

# 定义要安装的目录
set(INSTALL_DIRS
    face_age_detection
    face_landmarks_detection
    hand_gesture_detection
    hand_lmk_detection
    tros_ai_fusion
    tros_ai_fusion_msgs
    tros_lowpass_filter
    tros_perception_render
)

# 安装目录并设置权限
foreach(dir ${INSTALL_DIRS})
    install(DIRECTORY ${dir}
            DESTINATION ${install_parent_dir}
            USE_SOURCE_PERMISSIONS
            FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                             GROUP_READ GROUP_EXECUTE
                             WORLD_READ WORLD_EXECUTE)
endforeach()

# 最后调用 ament_package()
ament_package()