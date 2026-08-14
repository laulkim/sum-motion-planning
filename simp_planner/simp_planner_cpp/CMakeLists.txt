cmake_minimum_required(VERSION 3.16)
project(simp_planner_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(
  SIMP_PLANNER_ENABLE_FMA
  "Use hardware FMA for planner polynomial and motion integration kernels"
  ON
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simp_planner_msgs REQUIRED)

add_library(simp_planner_core STATIC
  src/core.cpp
  src/runtime.cpp
)
target_include_directories(simp_planner_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(simp_planner_core PRIVATE
  -Wall -Wextra -Wpedantic -O3 -ffp-contract=off
)
if(SIMP_PLANNER_ENABLE_FMA)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"
      AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    # Keep contraction disabled for ordinary a*b+c expressions so enabling
    # this option only changes the explicitly written multiply-add kernels.
    target_compile_options(simp_planner_core PRIVATE -mfma)
  else()
    message(WARNING
      "SIMP_PLANNER_ENABLE_FMA requested, but this compiler/architecture "
      "does not support the configured FMA flags; using portable math")
  endif()
endif()

add_executable(planner_node_cpp
  src/planner_node.cpp
)
target_link_libraries(planner_node_cpp
  simp_planner_core
)
ament_target_dependencies(planner_node_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  simp_planner_msgs
)
target_compile_options(planner_node_cpp PRIVATE
  -Wall -Wextra -Wpedantic -O3
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_simp_planner_core
    test/test_core.cpp
  )
  target_link_libraries(test_simp_planner_core
    simp_planner_core
  )
  target_compile_options(test_simp_planner_core PRIVATE
    -Wall -Wextra -Wpedantic
  )
  add_test(NAME simp_planner_core_tests COMMAND test_simp_planner_core)
endif()

install(TARGETS
  planner_node_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
