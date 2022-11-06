cmake_minimum_required(VERSION 3.5)
project(rmoss_gz_base)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ignition package
# Fortress
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

message(STATUS "Compiling against Ignition Fortress")


# include
include_directories(include)

# generate rmoss_gz_base lib
add_library(${PROJECT_NAME} SHARED
  src/gz_module/gz_chassis_actuator.cpp
  src/gz_module/gz_gimbal_actuator.cpp
  src/gz_module/gz_shoot_actuator.cpp
  src/gz_module/gz_gimbal_imu.cpp
  src/gz_module/gz_gimbal_encoder.cpp
  src/gz_module/gz_odometry.cpp
  src/gz_module/gz_light_bar_cmd.cpp
  src/pid.cpp
  src/chassis_controller.cpp
  src/gimbal_controller.cpp
  src/shooter_controller.cpp
  src/odometry_publisher.cpp
  src/rmua19_robot_base_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rmoss_interfaces
  tf2_ros
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

# nodes
add_executable(rmua19_robot_base src/rmua19_robot_base_main.cpp)
target_link_libraries(rmua19_robot_base ${PROJECT_NAME})

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rmoss_gz_base::Rmua19RobotBaseNode")

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

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(ignition-msgs${IGN_MSGS_VER})
ament_export_dependencies(ignition-transport${IGN_TRANSPORT_VER})
ament_package()
