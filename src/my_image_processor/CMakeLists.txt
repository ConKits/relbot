cmake_minimum_required(VERSION 3.8)
project(my_image_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)  
find_package(OpenCV REQUIRED)     

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}       
  ${cv_bridge_INCLUDE_DIRS}    
)

# Define executable
add_executable(image_processor 
    src/image_processor.cpp
)

# Link dependencies to the executable
target_link_libraries(image_processor
  ${OpenCV_LIBS}  
)

# Link dependencies using ament
ament_target_dependencies(image_processor 
    rclcpp 
    sensor_msgs 
    cv_bridge 
    OpenCV  
)

# Install executable
install(TARGETS
    image_processor
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
