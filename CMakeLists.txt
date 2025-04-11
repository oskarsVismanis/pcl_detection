cmake_minimum_required(VERSION 3.8)
project(pcl_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# find_package(moveit_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(shape_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

add_executable(perceive_surfaces src/perceive_surfaces.cpp)

target_include_directories(perceive_surfaces PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(perceive_surfaces PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  perceive_surfaces
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_msgs
  pcl_conversions
)

add_executable(pcl_tutorial src/pcl_tutorial.cpp)
ament_target_dependencies(
  pcl_tutorial
  rclcpp
  pcl_ros
  pcl_msgs
  pcl_conversions
  ament_index_cpp
)

add_executable(save_data src/save_data.cpp)
ament_target_dependencies(
  save_data
  rclcpp
  pcl_ros
  pcl_msgs
  pcl_conversions
  sensor_msgs
)

install(TARGETS 
  perceive_surfaces
  pcl_tutorial
  save_data
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  data/
  DESTINATION share/${PROJECT_NAME}/data
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
