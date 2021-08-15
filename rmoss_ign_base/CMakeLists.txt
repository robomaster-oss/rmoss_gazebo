cmake_minimum_required(VERSION 3.5)
project(rmoss_ign_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ros package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_ign_bridge REQUIRED)

# ignition package
find_package(ignition-transport10 REQUIRED)
set(IGN_TRANSPORT_VER 10)
find_package(ignition-msgs7 REQUIRED)
set(IGN_MSGS_VER 7)

# include
include_directories(include)

# generate rmoss_ign_base lib
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    rmoss_interfaces
    tf2_ros
    ros_ign_bridge
)
target_link_libraries(${PROJECT_NAME}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

#nodes
add_executable(rmua19_robot_base nodes/rmua19_robot_base_node.cpp)
target_link_libraries(rmua19_robot_base ${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
      rmua19_robot_base
  DESTINATION lib/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS
        scripts/test_chassis_cmd.py
        scripts/test_gimbal_cmd.py
        scripts/test_gimbal_cmd2.py
        scripts/test_shoot_cmd.py
    DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rmoss_interfaces)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(ros_ign_bridge)
ament_export_dependencies(ignition-transport10)
ament_export_dependencies(ignition-msgs7)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
