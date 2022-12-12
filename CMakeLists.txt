cmake_minimum_required(VERSION 3.8)
project(test_multi_nodes_multi_pub_sub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(multi_nodes_multi_pub src/multi_nodes_multi_pub.cpp)
ament_target_dependencies(multi_nodes_multi_pub rclcpp std_msgs)

add_executable(multi_nodes_multi_sub src/multi_nodes_multi_sub.cpp)
ament_target_dependencies(multi_nodes_multi_sub rclcpp std_msgs)

install(TARGETS
  multi_nodes_multi_pub
  multi_nodes_multi_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
