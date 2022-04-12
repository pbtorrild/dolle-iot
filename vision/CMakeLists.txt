cmake_minimum_required(VERSION 3.5)
project(vision)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
link_directories(include)
find_package(rcpputils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(v4l2_camera REQUIRED)
find_package(image_transport REQUIRED)

ament_export_dependencies(
  sensor_msgs
)

add_executable(talker src/basic_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/basic_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(limcheck src/limcheck.cpp)
ament_target_dependencies(limcheck rclcpp std_msgs OpenCV image_transport cv_bridge )

add_executable(camera src/camera.cpp)
ament_target_dependencies(camera rclcpp std_msgs OpenCV sensor_msgs )

#install executionables here
install(TARGETS
  talker
  listener
  limcheck
  camera
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
