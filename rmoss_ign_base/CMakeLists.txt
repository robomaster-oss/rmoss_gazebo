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
find_package(rmoss_interfaces REQUIRED)


# ignition package
find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)

# include
include_directories(include)

# generate rmoss_ign_base lib
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
add_executable(simple_gimbal_controller nodes/simple_gimbal_controller_node.cpp)
target_link_libraries(simple_gimbal_controller ${PROJECT_NAME})

add_executable(gimbal_state_publisher nodes/gimbal_state_publisher_node.cpp)
target_link_libraries(gimbal_state_publisher ${PROJECT_NAME})

add_executable(simple_chassis_controller nodes/simple_chassis_controller_node.cpp)
target_link_libraries(simple_chassis_controller ${PROJECT_NAME})

add_executable(simple_shooter_controller nodes/simple_shooter_controller_node.cpp)
target_link_libraries(simple_shooter_controller ${PROJECT_NAME})

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
      gimbal_state_publisher
      simple_gimbal_controller
      simple_chassis_controller 
      simple_shooter_controller
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
ament_export_dependencies(rmoss_interfaces)

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
