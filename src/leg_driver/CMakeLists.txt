cmake_minimum_required(VERSION 3.8)
project(leg_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

find_package(pkgConfig REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(leg_driver
  src/main.cpp
  src/serialnode.cpp
  src/usb_cdc
  src/package_comm.cpp
)


target_include_directories(leg_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/leg_driver>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(leg_driver
  rclcpp
  robot_interfaces
  serial)

install(TARGETS
leg_driver
DESTINATION lib/${PROJECT_NAME})

ament_package()
