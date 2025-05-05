cmake_minimum_required(VERSION 3.8)
project(pcl_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_library(pcl_detection src/pcl_detection.cpp)

target_include_directories(pcl_detection 
    PUBLIC
    #${CMAKE_CURRENT_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(pcl_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_compile_definitions(pcl_detection PUBLIC "PCL_DETECTION_BUILDING_LIBRARY")

ament_target_dependencies(
  pcl_detection PUBLIC
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_ros
  pcl_msgs
  pcl_conversions
)

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
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_ros
  pcl_msgs
  pcl_conversions
)

target_link_libraries(perceive_surfaces
  pcl_detection
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

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  pcl_detection
  perceive_surfaces
  pcl_tutorial
  save_data
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_export_libraries(pcl_detection)

ament_package()
