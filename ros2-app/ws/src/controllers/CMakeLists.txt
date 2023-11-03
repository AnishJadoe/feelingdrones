cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# Needed to find the .so file of px4_ros_com package
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

########### Add include directory for Eigen ###################
include_directories(
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)
  
add_executable(position_reference src/position_reference.cpp)
ament_target_dependencies(position_reference rclcpp geometry_msgs px4_msgs  px4_ros_com Eigen3)

add_executable(position_hover src/position_hover.cpp)
ament_target_dependencies(position_hover rclcpp geometry_msgs px4_msgs  px4_ros_com Eigen3)

add_executable(tactile_controller src/tactile_controller.cpp)
ament_target_dependencies(tactile_controller rclcpp std_msgs geometry_msgs px4_msgs custom_msgs px4_ros_com Eigen3)

add_executable(open_loop_perch src/open_loop_perch.cpp)
ament_target_dependencies(open_loop_perch rclcpp std_msgs geometry_msgs px4_msgs  px4_ros_com Eigen3)

target_include_directories(position_reference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/controllers>
  $<INSTALL_INTERFACE:include>)
target_compile_features(position_reference PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(position_hover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/controllers>
  $<INSTALL_INTERFACE:include>)
target_compile_features(position_hover PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(tactile_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/controllers>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tactile_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(open_loop_perch PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/controllers>
  $<INSTALL_INTERFACE:include>)
target_compile_features(open_loop_perch PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS position_reference
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS position_hover
DESTINATION lib/${PROJECT_NAME})

install(TARGETS tactile_controller
DESTINATION lib/${PROJECT_NAME})

install(TARGETS open_loop_perch
DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
