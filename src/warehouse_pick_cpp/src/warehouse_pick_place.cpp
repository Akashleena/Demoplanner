#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "warehouse_pick_place",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  auto const logger = rclcpp::get_logger("warehouse_pick_place");
  
  // Create MoveGroup interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "panda_arm");
  
  // Initialize visual tools
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node, "panda_link0", "warehouse_pick_place",
      move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  
  RCLCPP_INFO(logger, "=== Warehouse Pick and Place Demo ===");
  // ================================================================
// PHASE 0: Move to Safe Start Configuration
// ================================================================
RCLCPP_INFO(logger, "Phase 0: Moving to safe start configuration...");

std::vector<double> safe_joint_values = {
    0.0,      // panda_joint1
    -0.785,   // panda_joint2
    0.0,      // panda_joint3
    -2.356,   // panda_joint4
    0.0,      // panda_joint5
    1.571,    // panda_joint6
    0.785     // panda_joint7
};
move_group.setJointValueTarget(safe_joint_values);

moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
bool safe_success = (move_group.plan(safe_plan) == 
                     moveit::core::MoveItErrorCode::SUCCESS);

if (safe_success) {
    RCLCPP_INFO(logger, "✓ Safe start plan successful");
    move_group.execute(safe_plan);
    rclcpp::sleep_for(std::chrono::seconds(2));
} else {
    RCLCPP_ERROR(logger, "✗ Could not reach safe start!");
    rclcpp::shutdown();
    return 1;
}

  // ================================================================
  // PHASE 1: Move to approach position (above center shelf)
  // ================================================================
  RCLCPP_INFO(logger, "Phase 1: Planning to approach position...");
  
  geometry_msgs::msg::Pose approach_pose;
  approach_pose.position.x = 0.7;
  approach_pose.position.y = 0.0;
  approach_pose.position.z = 0.95;  // 10cm above target
  // Gripper pointing down (quaternion)
  approach_pose.orientation.x = 1.0;
  approach_pose.orientation.y = 0.0;
  approach_pose.orientation.z = 0.0;
  approach_pose.orientation.w = 0.0;
  
  move_group.setPoseTarget(approach_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
  bool success = (move_group.plan(approach_plan) == 
                  moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(logger, "✓ Approach plan successful");
    visual_tools.publishAxisLabeled(approach_pose, "approach");
    visual_tools.trigger();
    move_group.execute(approach_plan);
  } else {
    RCLCPP_ERROR(logger, "✗ Approach planning failed!");
    rclcpp::shutdown();
    return 1;
  }
  
  // ================================================================
  // PHASE 2: Cartesian path DOWN to grasp (VERTICAL APPROACH!)
  // ================================================================
  RCLCPP_INFO(logger, "Phase 2: Cartesian path down to grasp...");
  
  std::vector<geometry_msgs::msg::Pose> grasp_waypoints;
  
  // Start from current position
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  grasp_waypoints.push_back(current_pose);
  
  // Waypoint: Grasp position (straight down)
  geometry_msgs::msg::Pose grasp_pose = current_pose;
  grasp_pose.position.z = 0.85;  // At item height
  grasp_waypoints.push_back(grasp_pose);
  
  // Compute Cartesian path
  const double eef_step = 0.01;  // 1cm resolution
  moveit_msgs::msg::RobotTrajectory grasp_trajectory;
  double grasp_fraction = move_group.computeCartesianPath(
      grasp_waypoints, eef_step, grasp_trajectory);
  
  RCLCPP_INFO(logger, "Cartesian grasp path: %.2f%% achieved", 
              grasp_fraction * 100.0);
  
  if (grasp_fraction > 0.9) {
    visual_tools.publishPath(grasp_waypoints, rviz_visual_tools::LIME_GREEN, 
                            rviz_visual_tools::SMALL);
    visual_tools.publishAxisLabeled(grasp_pose, "grasp");
    visual_tools.trigger();
    move_group.execute(grasp_trajectory);
    RCLCPP_INFO(logger, "✓ Grasped item!");
  } else {
    RCLCPP_ERROR(logger, "✗ Cartesian grasp path failed!");
    rclcpp::shutdown();
    return 1;
  }
  
  // TODO: Close gripper here (if you have one)
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  // ================================================================
  // PHASE 3: Cartesian path UP to lift
  // ================================================================
  RCLCPP_INFO(logger, "Phase 3: Lifting item...");
  
  std::vector<geometry_msgs::msg::Pose> lift_waypoints;
  current_pose = move_group.getCurrentPose().pose;
  lift_waypoints.push_back(current_pose);
  
  geometry_msgs::msg::Pose lift_pose = current_pose;
  lift_pose.position.z = 1.0;  // Lift up
  lift_waypoints.push_back(lift_pose);
  
  moveit_msgs::msg::RobotTrajectory lift_trajectory;
  double lift_fraction = move_group.computeCartesianPath(
      lift_waypoints, eef_step, lift_trajectory);
  
  RCLCPP_INFO(logger, "Cartesian lift path: %.2f%% achieved", 
              lift_fraction * 100.0);
  
  if (lift_fraction > 0.9) {
    move_group.execute(lift_trajectory);
    RCLCPP_INFO(logger, "✓ Item lifted!");
  } else {
    RCLCPP_ERROR(logger, "✗ Lift path failed!");
  }
  
  // ================================================================
  // PHASE 4: Move to place location (above right shelf)
  // ================================================================
  RCLCPP_INFO(logger, "Phase 4: Moving to place location...");
  
  geometry_msgs::msg::Pose place_approach_pose;
  place_approach_pose.position.x = 0.7;
  place_approach_pose.position.y = -0.6;  // Right shelf
  place_approach_pose.position.z = 1.0;
  place_approach_pose.orientation = approach_pose.orientation;
  
  move_group.setPoseTarget(place_approach_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan place_approach_plan;
  success = (move_group.plan(place_approach_plan) == 
             moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    visual_tools.publishAxisLabeled(place_approach_pose, "place_approach");
    visual_tools.trigger();
    move_group.execute(place_approach_plan);
    RCLCPP_INFO(logger, "✓ Moved to place location");
  } else {
    RCLCPP_ERROR(logger, "✗ Place approach failed!");
  }
  
  // ================================================================
  // PHASE 5: Cartesian path DOWN to place
  // ================================================================
  RCLCPP_INFO(logger, "Phase 5: Placing item...");
  
  std::vector<geometry_msgs::msg::Pose> place_waypoints;
  current_pose = move_group.getCurrentPose().pose;
  place_waypoints.push_back(current_pose);
  
  geometry_msgs::msg::Pose place_pose = current_pose;
  place_pose.position.z = 0.85;  // Place on shelf
  place_waypoints.push_back(place_pose);
  
  moveit_msgs::msg::RobotTrajectory place_trajectory;
  double place_fraction = move_group.computeCartesianPath(
      place_waypoints, eef_step, place_trajectory);
  
  RCLCPP_INFO(logger, "Cartesian place path: %.2f%% achieved", 
              place_fraction * 100.0);
  
  if (place_fraction > 0.9) {
    visual_tools.publishPath(place_waypoints, rviz_visual_tools::RED, 
                            rviz_visual_tools::SMALL);
    visual_tools.trigger();
    move_group.execute(place_trajectory);
    RCLCPP_INFO(logger, "✓ Item placed!");
  } else {
    RCLCPP_ERROR(logger, "✗ Place path failed!");
  }
  
  // TODO: Open gripper here
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  // ================================================================
  // PHASE 6: Cartesian path UP to retreat
  // ================================================================
  RCLCPP_INFO(logger, "Phase 6: Retreating...");
  
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  current_pose = move_group.getCurrentPose().pose;
  retreat_waypoints.push_back(current_pose);
  
  geometry_msgs::msg::Pose retreat_pose = current_pose;
  retreat_pose.position.z = 1.0;
  retreat_waypoints.push_back(retreat_pose);
  
  moveit_msgs::msg::RobotTrajectory retreat_trajectory;
  double retreat_fraction = move_group.computeCartesianPath(
      retreat_waypoints, eef_step, retreat_trajectory);
  
  if (retreat_fraction > 0.9) {
    move_group.execute(retreat_trajectory);
    RCLCPP_INFO(logger, "✓ Retreat complete!");
  }
  
  RCLCPP_INFO(logger, "=== Pick and Place Complete! ===");
  
  rclcpp::shutdown();
  return 0;
}