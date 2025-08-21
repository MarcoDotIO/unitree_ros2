cmake_minimum_required(VERSION 3.5)
project(dance_orchestrator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20 (required for dance orchestrator features)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../example/src/include)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add executable
add_executable(dance_orchestrator
  src/main.cpp
  src/dance_orchestrator.cpp
  src/move_registry.cpp
  src/cli_interface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../example/src/src/common/ros2_sport_client.cpp
)

# Set C++20 standard for the target
target_compile_features(dance_orchestrator PRIVATE cxx_std_20)

# Target include directories
target_include_directories(dance_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../../example/src/include
)

# Link dependencies
ament_target_dependencies(dance_orchestrator ${DEPENDENCY_LIST})
target_link_libraries(dance_orchestrator nlohmann_json::nlohmann_json)

# Install targets
install(TARGETS dance_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
