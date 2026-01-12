#include <memory>
#include <vector>
#include <limits>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit_msgs/msg/constraints.hpp>              // ADDED
#include <moveit_msgs/msg/orientation_constraint.hpp>   // ADDED
#include "utils/config_loader.hpp"
#include "utils/experiment_logger.hpp"
// ============================================================================
// VISUALIZATION HELPERS
// ============================================================================

void publish_goal_marker(
    rclcpp::Node::SharedPtr node,
    const geometry_msgs::msg::Pose& goal_pose)
{
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "/goal_marker", 10
    );
    
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = node->get_clock()->now();
    marker.ns = "goal_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = goal_pose;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    
    marker.color.r = 1.0;  // RED for goal
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = rclcpp::Duration::from_seconds(60);
    
    marker_pub->publish(marker);
}

void visualize_trajectory_waypoints(
    rclcpp::Node::SharedPtr node,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& color_name = "blue")
{
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/trajectory_waypoints", 10
    );
    
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Get robot model and state
    const auto& joint_traj = plan.trajectory.joint_trajectory;
    
    // For each waypoint in trajectory
    for (size_t i = 0; i < joint_traj.points.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = node->get_clock()->now();
        marker.ns = "waypoints_" + color_name;
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // We need to compute forward kinematics to get end-effector position
        // For now, just show markers along the trajectory
        // (Full FK requires robot model, simplified here)
        
        marker.pose.position.x = 0.5 + i * 0.02;  // Placeholder - will fix below
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5 + i * 0.01;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        
        // Color based on name
        if (color_name == "blue") {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else if (color_name == "green") {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else {  // red
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 0.7;
        
        marker.lifetime = rclcpp::Duration::from_seconds(60);
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub->publish(marker_array);
    
    RCLCPP_INFO(rclcpp::get_logger("viz"), 
                "Published %zu waypoint markers (%s)",
                joint_traj.points.size(), color_name.c_str());
}

// ============================================================================
// TRAJECTORY SCORING (same as before)
// ============================================================================

struct TrajectoryScore {
    bool valid;
    bool dense_collision_ok;
    double path_length;
    double smoothness;
    double total_cost;
    
    TrajectoryScore() 
        : valid(false), dense_collision_ok(true), 
          path_length(0.0), smoothness(0.0), total_cost(std::numeric_limits<double>::max()) {}
};

TrajectoryScore score_trajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    TrajectoryScore score;
    score.valid = true;
    
    const auto& joint_traj = plan.trajectory.joint_trajectory;
    
    // Path length
    score.path_length = 0.0;
    for (size_t i = 1; i < joint_traj.points.size(); ++i) {
        double segment = 0.0;
        for (size_t j = 0; j < joint_traj.points[i].positions.size(); ++j) {
            double delta = joint_traj.points[i].positions[j] - 
                          joint_traj.points[i-1].positions[j];
            segment += delta * delta;
        }
        score.path_length += std::sqrt(segment);
    }
    
    // Smoothness
    score.smoothness = 0.0;
    if (joint_traj.points.size() >= 3) {
        for (size_t i = 1; i < joint_traj.points.size() - 1; ++i) {
            for (size_t j = 0; j < joint_traj.points[i].positions.size(); ++j) {
                double second_diff = 
                    joint_traj.points[i+1].positions[j] - 
                    2*joint_traj.points[i].positions[j] + 
                    joint_traj.points[i-1].positions[j];
                score.smoothness += second_diff * second_diff;
            }
        }
    }
    
    const double alpha = 1.0;
    const double beta = 0.5;
    score.total_cost = alpha * score.path_length + beta * score.smoothness;
    
    if (!score.dense_collision_ok) {
        score.total_cost += 1000.0;
    }
    
    return score;
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "warehouse_systems_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  auto const logger = rclcpp::get_logger("systems_demo");
  
  // ================================================================
  // LOAD CONFIG FROM YAML
  // ================================================================
  std::string config_path = "../config/experiment.yaml";
  ExperimentConfig config;
  
  try {
    config = ExperimentConfig::load(config_path);
    config.print();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to load config: %s", e.what());
    RCLCPP_ERROR(logger, "Using hardcoded defaults instead");
    
    // Fallback to hardcoded values
    config.multi_start_enabled = true;
    config.num_attempts = 6;
    config.planning_time_sec = 30.0;
    config.goal_x = 0.5;
    config.goal_y = 0.0;
    config.goal_z = 0.80;
    // config.start_joints = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    // config.start_joints = {0.0, 0.5, 0.0, -1.5, 0.0, 2.0, 0.785};
    // config.start_joints = {0.212248, -0.50645, -0.180782, -2.38856, -0.0916522, 1.88693, 0.0823912};
    config.start_joints = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    config.output_dir = "/home/aleenatron/ws_moveit/results";
    config.csv_filename = "constraint_1deg.csv";
  }
  
  // ================================================================
  // SETUP CSV LOGGER
  // ================================================================
  std::string log_path = config.output_dir + "/" + config.csv_filename;
  ExperimentLogger logger_csv(log_path);
  RCLCPP_INFO(logger, "Logging to: %s", log_path.c_str());
  
  // ================================================================
  // SETUP MOVE GROUP
  // ================================================================
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "panda_arm");
  
  RCLCPP_INFO(logger, "=== Warehouse Systems Demo ===");
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // PHASE 0: Safe Start (FROM CONFIG)
  // ================================================================
  RCLCPP_INFO(logger, "[Phase 0] Moving to safe start...");
  move_group.setJointValueTarget(config.start_joints);
  
  moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
  if (move_group.plan(safe_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "✗ Safe start failed!");
    rclcpp::shutdown();
    return 1;
  }
  
  move_group.execute(safe_plan);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(logger, "✓ Safe start complete");
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // GOAL SETUP (FROM CONFIG)
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 0] Setting up goal...");
//   move_group.setPositionTarget(config.goal_x, config.goal_y, config.goal_z, "panda_hand");
  geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = config.goal_x;
    goal_pose.position.y = config.goal_y;
    goal_pose.position.z = config.goal_z;
    goal_pose.orientation.x = 1.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;
    move_group.setPoseTarget(goal_pose, "panda_link8");
  RCLCPP_INFO(logger, "Goal position: (%.2f, %.2f, %.2f)", 
              config.goal_x, config.goal_y, config.goal_z);
  
  // ================================================================
  // ORIENTATION CONSTRAINT: Top-down gripper (ADDED)
  // ================================================================
  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "top_down_gripper";

  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "panda_link8";
  ocm.header.frame_id = "panda_link0";

  // Top-down orientation: gripper Z-axis points DOWN
  // This is 180° rotation about X-axis
  // Quaternion: w=0, x=1, y=0, z=0
  ocm.orientation.w = 0.0;
  ocm.orientation.x = 1.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;

  // Tolerances in radians
  ocm.absolute_x_axis_tolerance = 0.017;   // ~15° tilt allowed
  ocm.absolute_y_axis_tolerance = 0.017;   // ~15° tilt allowed
  ocm.absolute_z_axis_tolerance = 3.14;  // Free rotation about vertical (yaw)
  ocm.weight = 1.0;

  path_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(path_constraints);

  RCLCPP_INFO(logger, "Orientation: TOP-DOWN constrained (±1°)");
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // STAGE 1: Multi-Start Planning (FROM CONFIG)
  // ================================================================
  const int K = config.multi_start_enabled ? config.num_attempts : 1;
  move_group.setPlanningTime(config.planning_time_sec);
  
  RCLCPP_INFO(logger, "[Stage 1] Multi-start planning (K=%d)...", K);
  RCLCPP_INFO(logger, " ");
  
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> candidates;
  std::vector<TrajectoryScore> scores;
  
  for (int i = 0; i < K; ++i) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    // Measure planning time
    auto t_start = std::chrono::high_resolution_clock::now();
    auto result = move_group.plan(plan);
    auto t_end = std::chrono::high_resolution_clock::now();
    double planning_time = std::chrono::duration<double>(t_end - t_start).count();
    
    TrajectoryScore score;
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      score = score_trajectory(plan);
      candidates.push_back(plan);
      
      // LOG SUCCESS TO CSV
      logger_csv.log_attempt(0, i, true, planning_time, 
                            score.path_length, score.smoothness, score.total_cost);
      
      RCLCPP_INFO(logger, "Attempt %d | valid=1 | time=%.3fs | score=%.2f | len=%.2f | smooth=%.2f",
                  i, planning_time, score.total_cost, score.path_length, score.smoothness);
      
    } else {
      score.valid = false;
      
      // LOG FAILURE TO CSV
      logger_csv.log_attempt(0, i, false, planning_time, 0, 0, 999);
      
      RCLCPP_INFO(logger, "Attempt %d | valid=0 | time=%.3fs", i, planning_time);
    }
    
    scores.push_back(score);
  }
  
  logger_csv.flush();  // Save CSV immediately
  
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // STAGE 2: Select Best
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 2] Selecting best candidate...");
  
  if (candidates.empty()) {
    RCLCPP_ERROR(logger, "✗ No valid plans found!");
    RCLCPP_ERROR(logger, "RRT struggles with orientation constraints - this is expected!");
    rclcpp::shutdown();
    return 1;
  }
  
  std::vector<int> valid_indices;
  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores[i].valid) {
      valid_indices.push_back(i);
    }
  }
  
  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < valid_indices.size(); ++i) {
    int attempt_idx = valid_indices[i];
    if (scores[attempt_idx].total_cost < best_cost) {
      best_cost = scores[attempt_idx].total_cost;
      best_idx = i;
    }
  }
  
  RCLCPP_INFO(logger, "Best attempt: %d (cost=%.2f)", valid_indices[best_idx], best_cost);
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // CLEANUP: Clear constraints (ADDED)
  // ================================================================
  move_group.clearPathConstraints();
  
  // ================================================================
  // DONE (Skip execution for now)
  // ================================================================
  RCLCPP_INFO(logger, "=== Planning Complete! ===");
  RCLCPP_INFO(logger, "Results saved to: %s", log_path.c_str());
  
  rclcpp::shutdown();
  return 0;
}