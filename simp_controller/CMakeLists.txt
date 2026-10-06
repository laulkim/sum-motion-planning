cmake_minimum_required(VERSION 3.16)
project(simp_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simp_planner_msgs REQUIRED)

add_executable(tracking_controller_node src/tracking_controller_node.cpp)
target_include_directories(tracking_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(tracking_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  simp_planner_msgs
)
target_compile_options(tracking_controller_node PRIVATE -Wall -Wextra -Wpedantic -O2)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_tracking_index test/test_tracking_index.cpp)
  target_include_directories(test_tracking_index PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_options(test_tracking_index PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME tracking_index_tests COMMAND test_tracking_index)
endif()

install(TARGETS
  tracking_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
