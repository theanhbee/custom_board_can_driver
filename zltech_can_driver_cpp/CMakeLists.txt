cmake_minimum_required(VERSION 3.8)
project(zltech_can_driver_cpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# Thu vien dung chung: giai ma loi + giao thuc CAN (Waveshare / SocketCAN) + tien ich dong lenh. Khong phu thuoc ROS.
add_library(zltech_can_core STATIC
  src/odrive_can_errors.cpp
  src/protocol.cpp
  src/cli.cpp)
target_include_directories(zltech_can_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(zltech_can_core Threads::Threads)
set_target_properties(zltech_can_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Node ROS 2
add_executable(can_bridge src/can_bridge.cpp)
target_link_libraries(can_bridge zltech_can_core)
ament_target_dependencies(can_bridge rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2_ros)

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop zltech_can_core)
ament_target_dependencies(teleop rclcpp geometry_msgs std_msgs)

# Tool dong lenh (khong can ROS)
foreach(tool calib clear_errors drive_test error_monitor)
  add_executable(${tool} src/${tool}.cpp)
  target_link_libraries(${tool} zltech_can_core)
endforeach()

install(TARGETS can_bridge teleop calib clear_errors drive_test error_monitor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odrive_can_errors test/test_odrive_can_errors.cpp)
  target_link_libraries(test_odrive_can_errors zltech_can_core)
endif()

ament_package()
