cmake_minimum_required(VERSION 3.5)
project(rmoss_ign_bridge)

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
find_package(ros_ign_bridge REQUIRED)

# ignition package
# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-transport10 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
endif()

# include
include_directories(include)

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

# pose_bridge nodes
add_executable(pose_bridge  src/pose_bridge_node.cpp src/pose_bridge_main.cpp)
target_link_libraries(pose_bridge
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)
ament_target_dependencies(pose_bridge ${dependencies})
ament_target_dependencies(pose_bridge ros_ign_bridge)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
# install(TARGETS ${PROJECT_NAME}
#     EXPORT ${PROJECT_NAME}
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     RUNTIME DESTINATION bin
#     INCLUDES DESTINATION include
# )

# Install executables
install(TARGETS
      pose_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_dependencies(ignition-msgs${IGN_MSGS_VER})
ament_export_dependencies(ignition-transport${IGN_TRANSPORT_VER})
ament_package()
