cmake_minimum_required(VERSION 3.8)
project(relbot_sequence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

add_executable(relbot_sequence_controller
src/relbot_sequence_controller.cpp)

ament_target_dependencies(relbot_sequence_controller
    rclcpp
    example_interfaces
    sensor_msgs
    cv_bridge
)

install(
    TARGETS
        relbot_sequence_controller
        DESTINATION lib/${PROJECT_NAME}
)

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

# # install the launch directory
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )
