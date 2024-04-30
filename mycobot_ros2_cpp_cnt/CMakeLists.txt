cmake_minimum_required(VERSION 3.8)
project(mycobot_ros2_cpp_cnt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mycobot_interfaces REQUIRED)



add_executable(marker_creater src/marker_creater.cpp)
add_executable(grip_test src/grip_test.cpp)

target_include_directories(marker_creater PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(marker_creater PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  marker_creater
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "vision_msgs"
)

ament_target_dependencies(
  grip_test
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "vision_msgs"
  "sensor_msgs"
  "mycobot_interfaces"
)

install(TARGETS marker_creater grip_test
  DESTINATION lib/${PROJECT_NAME})

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
