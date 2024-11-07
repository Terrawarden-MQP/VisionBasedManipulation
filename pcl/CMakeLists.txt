cmake_minimum_required(VERSION 3.8)
project(pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters io)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  include
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(transform_pointcloud src/transform_pointcloud.cpp)
target_link_libraries(transform_pointcloud ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_eigen_LIBRARIES})

ament_target_dependencies(	
  transform_pointcloud
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen  
)

# Install the executables
install(TARGETS
  transform_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(surface_normal src/surface_normal.cpp)
target_link_libraries(surface_normal ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_eigen_LIBRARIES})
find_package(visualization_msgs REQUIRED)

add_executable(stitch_pointcloud src/stitch_pointcloud.cpp)
target_link_libraries(stitch_pointcloud ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_eigen_LIBRARIES})

ament_target_dependencies(
  surface_normal
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  PCL
  visualization_msgs
  geometry_msgs
)

# For stitch_pointcloud
ament_target_dependencies(
  stitch_pointcloud
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  PCL
)

# Install the executables
install(TARGETS
  surface_normal
  stitch_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(plane_segmentation src/plane_segmentation.cpp)
target_link_libraries(plane_segmentation ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_eigen_LIBRARIES})

ament_target_dependencies(
  plane_segmentation
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen  
  PCL
)

install(TARGETS
  plane_segmentation
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

# Testing 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
