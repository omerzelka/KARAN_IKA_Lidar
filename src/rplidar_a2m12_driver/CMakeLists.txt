cmake_minimum_required(VERSION 3.8)
project(rplidar_a2m12_driver)

# C++17: iç içe namespace ve modern atomik garantiler için
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # -O2: Jetson üzerinde parsing döngüsünün optimize edilmesi için
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(rplidar_node
  src/rplidar_driver_node.cpp
  src/serial_transport.cpp
)

target_include_directories(rplidar_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(rplidar_node
  rclcpp
  sensor_msgs
)

# Okuyucu thread'i için pthread
target_link_libraries(rplidar_node Threads::Threads)

install(TARGETS rplidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
