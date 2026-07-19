cmake_minimum_required(VERSION 3.8)
project(ika_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Algoritma kütüphanesi: Stanley + VFH+ + Kalman (ROS'tan bağımsız) ---
add_library(ika_algorithms
  src/stanley_controller.cpp
  src/vfh_plus.cpp
  src/kalman_filter.cpp
)
target_include_directories(ika_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Navigasyon düğümü: algoritmaları ROS boru hattında birleştirir ---
add_executable(ika_navigation_node
  src/ika_navigation_node.cpp
)
target_include_directories(ika_navigation_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ika_navigation_node ika_algorithms)

ament_target_dependencies(ika_navigation_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS ika_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Birim testleri (isteğe bağlı; colcon test ile çalışır) ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_algorithms test/test_algorithms.cpp)
    target_include_directories(test_algorithms PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_algorithms ika_algorithms)
  endif()
endif()

ament_package()
