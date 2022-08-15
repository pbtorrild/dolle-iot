cmake_minimum_required(VERSION 3.5)
project(limcheck)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
if(CMAKE_VERSION VERSION_LESS "3.10")
  cmake_policy(SET CMP0071 OLD)
else()
  cmake_policy(SET CMP0071 NEW)
endif()
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

find_package(BLAS REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(qml_ros2_plugin REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Quick Charts)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Quick Charts)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CleaningStamp.msg"
  "msg/ImagePos.msg"
  "msg/ObjectPos.msg"
)

set(srv_files
  "srv/StopSignal.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
set_property(SOURCE ${msg_files} ${srv_files} PROPERTY SKIP_AUTOGEN ON)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  sensor_msgs
  Qt${QT_VERSION_MAJOR}
  BLAS
)

add_executable(camera src/camera.cpp)
ament_target_dependencies(camera rclcpp std_msgs image_transport cv_bridge BLAS)
rosidl_target_interfaces(camera ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Gui install
include_directories(src/gui/include)
link_directories(src/gui/include)

set(GUI_SOURCES
        src/gui/main.cpp
        src/gui/qml.qrc
        
)

add_executable(gui ${GUI_SOURCES})

target_link_libraries(gui
  
  PRIVATE
  Qt${QT_VERSION_MAJOR}::Core 
  Qt${QT_VERSION_MAJOR}::Quick 
  Qt${QT_VERSION_MAJOR}::Charts 
  ${rclcpp_LIBRARIES} 
  ${std_msgs_LIBRARIES}
  ${qml_ros2_plugin_LIBRARIES}
  
)

#install executables here
install(TARGETS
  camera
  gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY data/ml_models/
DESTINATION include/ml_models)

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
