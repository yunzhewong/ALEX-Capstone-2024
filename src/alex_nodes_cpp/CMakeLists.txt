cmake_minimum_required(VERSION 3.8)
project(alex_nodes_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/simple_publisher.cpp)
add_executable(listener src/simple_subscriber.cpp)
add_executable(encoder_reader src/encoder_reader.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(encoder_reader rclcpp std_msgs)

install(TARGETS
  talker
  listener
  encoder_reader
  DESTINATION lib/${PROJECT_NAME})

ament_package()
