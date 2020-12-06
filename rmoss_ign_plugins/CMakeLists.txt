cmake_minimum_required(VERSION 3.5)
project(rmoss_ign_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find package
find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
set(IGN_PLUGIN_VER 1)
find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER 4)
find_package(ignition-common3 REQUIRED)
set(IGN_COMMON_VER 3)
find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)


# for plugins MecanumDrive
add_library(MecanumDrive SHARED src/mecanum_drive/MecanumDrive.cpp)

target_link_libraries(MecanumDrive
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)

# for plugins ProjectileShooter
add_library(ProjectileShooter SHARED src/projectile_shooter/ProjectileShooter.cpp)

target_link_libraries(ProjectileShooter
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)

# Install plugins
install(TARGETS MecanumDrive ProjectileShooter
    DESTINATION lib
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ignition.dsv.in")

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
