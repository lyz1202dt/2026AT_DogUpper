cmake_minimum_required(VERSION 3.8)
project(leg_calc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(leg_driver REQUIRED)

find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)

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

add_executable(leg_calc
src/main.cpp
src/param.cpp
src/leg.cpp
src/mynode.cpp
src/step.cpp
src/vmc.cpp)


target_include_directories(leg_calc
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/leg_calc>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${leg_driver_LIBRARIES})

target_link_libraries(leg_calc Eigen3::Eigen)

ament_target_dependencies(
  leg_calc
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  robot_interfaces
  leg_driver
)

install(TARGETS
leg_calc
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
