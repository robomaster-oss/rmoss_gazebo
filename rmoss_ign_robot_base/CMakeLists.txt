cmake_minimum_required(VERSION 3.5)
project(rmoss_ign_robot_base)

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
find_package(rmoss_interfaces REQUIRED)

# ignition package
find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)

# include
include_directories(include)

# generate rmua19_ignition_simulator lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    std_msgs
    geometry_msgs
    rmoss_interfaces
)
target_link_libraries(${PROJECT_NAME}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

#nodes
add_executable(gimbal_simple_controller nodes/gimbal_simple_controller_node.cpp)
target_link_libraries(gimbal_simple_controller ${PROJECT_NAME})

add_executable(gimbal_state_publisher nodes/gimbal_state_publisher_node.cpp)
target_link_libraries(gimbal_state_publisher ${PROJECT_NAME})

add_executable(chassis_simple_controller nodes/chassis_simple_controller_node.cpp)
target_link_libraries(chassis_simple_controller ${PROJECT_NAME})

add_executable(shooter_simple_controller nodes/shooter_simple_controller_node.cpp)
target_link_libraries(shooter_simple_controller ${PROJECT_NAME})

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
      gimbal_simple_controller
      gimbal_state_publisher
      chassis_simple_controller 
      shooter_simple_controller
  DESTINATION lib/${PROJECT_NAME} 
)

# Install executable scripts
install(PROGRAMS 
        scripts/test_control_chassis.py 
        scripts/test_control_gimbal.py
    DESTINATION lib/${PROJECT_NAME})

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
