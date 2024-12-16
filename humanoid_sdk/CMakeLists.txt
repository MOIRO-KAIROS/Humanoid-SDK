cmake_minimum_required(VERSION 3.8)
project(humanoid_sdk)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(pybind11 REQUIRED)

# pybind11_add_module(humanoid_sdk src/humanoid_sdk_wrapper.cpp)

include_directories(include)

# Build (각각 빌드 - 1. read_write_node, 2. read_write_multi)
add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
# add_executable(read_write_multi src/read_write_multi.cpp)
# ament_target_dependencies(read_write_multi
#   dynamixel_sdk_custom_interfaces
#   dynamixel_sdk
#   rclcpp
# )

# Install
install(TARGETS
  read_write_node
  DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS
#   read_write_multi
#   DESTINATION lib/${PROJECT_NAME}
# )

# Test
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

ament_package()