cmake_minimum_required(VERSION 3.8)
project(grasp_vision_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}) 

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

add_executable(extract_cluster src/extract_cluster.cpp)
ament_target_dependencies(extract_cluster rclcpp sensor_msgs geometry_msgs pcl_conversions pcl_msgs cv_bridge)

add_executable(optimal_grasp src/optimal_grasp.cpp)
ament_target_dependencies(optimal_grasp rclcpp sensor_msgs geometry_msgs pcl_conversions pcl_msgs visualization_msgs)

install(TARGETS
  extract_cluster
  optimal_grasp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
