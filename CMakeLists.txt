cmake_minimum_required(VERSION 3.8)
project(reach_ros_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(checksum_utils src/checksum_utils.cpp)
ament_target_dependencies(checksum_utils)

add_library(parser src/parser.cpp)
ament_target_dependencies(parser)
 
add_library(driver src/driver.cpp)
ament_target_dependencies(driver rclcpp sensor_msgs)

add_executable(nmea_serial_driver src/nmea_serial_driver.cpp src/driver.cpp)
target_link_libraries(nmea_serial_driver
  checksum_utils
  parser
  driver
  Boost::system
  Boost::thread
  Threads::Threads
  udev
)
ament_target_dependencies(nmea_serial_driver rclcpp sensor_msgs Boost)

target_include_directories(nmea_serial_driver
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

install(TARGETS
  checksum_utils
  parser
  driver
  nmea_serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   ${PROJECT_NAME}/waypoint_navigation.py
#   ${PROJECT_NAME}/tf_test_node.py
#   ${PROJECT_NAME}/key_pub.py
# DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
