cmake_minimum_required(VERSION 3.8)
project(warehouse_pick_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(warehouse_pick_place src/warehouse_pick_place.cpp)
add_executable(warehouse_systems_demo src/warehouse_systems_demo.cpp)
ament_target_dependencies(warehouse_pick_place
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)
ament_target_dependencies(warehouse_systems_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  visualization_msgs
)
add_executable(orientation_finder src/orientation_finder.cpp)
target_include_directories(orientation_finder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(orientation_finder PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(orientation_finder
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS orientation_finder
  DESTINATION lib/${PROJECT_NAME})
# Install
install(TARGETS
  warehouse_pick_place
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS warehouse_systems_demo DESTINATION lib/${PROJECT_NAME})

ament_package()