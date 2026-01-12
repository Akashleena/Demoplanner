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
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ================================================================
# UTILS LIBRARY
# ================================================================
add_library(experiment_utils
  src/utils/config_loader.cpp
  src/utils/experiment_logger.cpp
)

target_include_directories(experiment_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(experiment_utils
  yaml-cpp
)

# ================================================================
# EXECUTABLES
# ================================================================

# warehouse_pick_place (existing)
add_executable(warehouse_pick_place src/warehouse_pick_place.cpp)
ament_target_dependencies(warehouse_pick_place
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)
add_executable(find_topdown_joints src/find_topdown_joints.cpp)
ament_target_dependencies(find_topdown_joints rclcpp moveit_ros_planning_interface)
install(TARGETS find_topdown_joints DESTINATION lib/${PROJECT_NAME})
# warehouse_systems_demo (updated with utils)
add_executable(warehouse_systems_demo src/warehouse_systems_demo.cpp)
ament_target_dependencies(warehouse_systems_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  visualization_msgs
)
target_link_libraries(warehouse_systems_demo
  experiment_utils  # ← Links yaml-cpp + utils
)

# orientation_finder (existing)
add_executable(orientation_finder src/orientation_finder.cpp)
target_include_directories(orientation_finder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(orientation_finder PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(orientation_finder
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

# ================================================================
# INSTALL
# ================================================================
install(TARGETS
  warehouse_pick_place
  warehouse_systems_demo
  orientation_finder
  experiment_utils  # ← Install library too
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
