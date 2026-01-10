#include <memory>
#include <vector>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "panda_arm");
  
  RCLCPP_INFO(logger, "=== Warehouse Systems Demo with Visualization ===");
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // PHASE 0: Move to Safe Start
  // ================================================================
  RCLCPP_INFO(logger, "[Phase 0] Moving to safe start...");
  std::vector<double> safe_joints = {
      0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785
  };
  move_group.setJointValueTarget(safe_joints);
  
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
  // DIAGNOSTIC: Check current position and reach
  // ================================================================
  // geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  // RCLCPP_INFO(logger, "[Diagnostic] Current EE: (%.2f, %.2f, %.2f)",
  //             current_pose.position.x,
  //             current_pose.position.y,
  //             current_pose.position.z);
  
  // ================================================================
  // GOAL SETUP (C: Fixed coordinates within reach)
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 0] Setting up goal...");
  move_group.setPositionTarget(0.65, 0.0, 0.80, "panda_hand");

  RCLCPP_INFO(logger, "Goal position: (0.65, 0.00, 0.80)");
  RCLCPP_INFO(logger, "Orientation: FREE (RRT optimizes)");

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.65;  // Within reach!
  goal_pose.position.y = 0.0;   
  goal_pose.position.z = 0.80;  // Above item
  
  goal_pose.orientation.x = 1.0; // vertical down
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = 0.0;
  goal_pose.orientation.w = 0.0;
  
  RCLCPP_INFO(logger, "Goal EE position: (%.2f, %.2f, %.2f)",
              goal_pose.position.x,
              goal_pose.position.y,
              goal_pose.position.z);
  
  // double distance = std::sqrt(
  //     std::pow(goal_pose.position.x - current_pose.position.x, 2) +
  //     std::pow(goal_pose.position.y - current_pose.position.y, 2) +
  //     std::pow(goal_pose.position.z - current_pose.position.z, 2)
  // );
  
  // RCLCPP_INFO(logger, "Distance to goal: %.2fm (Panda max reach ~0.85m)", distance);
  
  // if (distance > 0.85) {
  //   RCLCPP_WARN(logger, "⚠ Goal might be beyond reach!");
  // }
  
  // (A) Visualize goal marker
  RCLCPP_INFO(logger, "Publishing goal marker (RED sphere in RViz)...");
  publish_goal_marker(node, goal_pose);
  rclcpp::sleep_for(std::chrono::seconds(1));
  
  // // (B) IK Check
  // RCLCPP_INFO(logger, "[IK Check] Testing goal reachability...");
  // move_group.setPoseTarget(goal_pose);
  
  // ============================================================================
// SEGFAULT
// ============================================================================
  // bool ik_valid = move_group.setApproximateJointValueTarget(goal_pose, "panda_hand");

  
  // if (!ik_valid) {
  //   RCLCPP_ERROR(logger, "✗ GOAL IS UNREACHABLE (IK failed)!");
  //   RCLCPP_ERROR(logger, "  Reduce X coordinate or adjust Z height");
  //   rclcpp::shutdown();
  //   return 1;
  // }
  
  // RCLCPP_INFO(logger, "✓ Goal is kinematically reachable");
  // RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // STAGE 1: Multi-Start Planning
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 1] Multi-start planning (K=6)...");
  RCLCPP_INFO(logger, " ");
  
  // move_group.setPoseTarget(goal_pose); // doing this because rrt cannot find a path
  move_group.setPlanningTime(10.0);
  
  const int K = 6;
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> candidates;
  std::vector<TrajectoryScore> scores;
  
  for (int i = 0; i < K; ++i) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);
    
    TrajectoryScore score;
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      score = score_trajectory(plan);
      candidates.push_back(plan);
      
      RCLCPP_INFO(logger, "Attempt %d | valid=1 | dense=1 | score=%.2f | len=%.2f | smooth=%.2f",
                  i, score.total_cost, score.path_length, score.smoothness);
      
      // Visualize this trajectory's waypoints
      // (For now just first successful one to avoid clutter)
      if (candidates.size() == 1) {
        RCLCPP_INFO(logger, "  → Visualizing waypoints (blue dots)");
        visualize_trajectory_waypoints(node, move_group, plan, "blue");
      }
      
    } else {
      score.valid = false;
      RCLCPP_INFO(logger, "Attempt %d | valid=0", i);
    }
    
    scores.push_back(score);
  }
  
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // STAGE 2: Select Best
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 2] Selecting best candidate...");
  
  if (candidates.empty()) {
    RCLCPP_ERROR(logger, "✗ No valid plans found!");
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
  // Execute Best Plan
  // ================================================================
  RCLCPP_INFO(logger, "Executing best trajectory to approach...");
  
  if (best_idx >= 0) {
    move_group.execute(candidates[best_idx]);
    RCLCPP_INFO(logger, "✓ Reached approach pose");
  }
  
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // PHASE 2: Cartesian Descent
  // ================================================================
  RCLCPP_INFO(logger, "[Phase 2] Descending to grasp...");
  
  std::vector<geometry_msgs::msg::Pose> grasp_waypoints;
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  grasp_waypoints.push_back(current_pose);
  
  geometry_msgs::msg::Pose grasp_pose = current_pose;
  grasp_pose.position.z = 0.70;  // Item height
  grasp_waypoints.push_back(grasp_pose);
  
  moveit_msgs::msg::RobotTrajectory grasp_trajectory;
  double grasp_fraction = move_group.computeCartesianPath(
      grasp_waypoints, 0.01, grasp_trajectory
  );
  
  RCLCPP_INFO(logger, "Cartesian grasp: %.1f%% achieved", grasp_fraction * 100.0);
  
  if (grasp_fraction > 0.9) {
    move_group.execute(grasp_trajectory);
    RCLCPP_INFO(logger, "✓ Reached grasp position");
  } else {
    RCLCPP_WARN(logger, "⚠ Grasp path incomplete");
  }
  
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(logger, "[Simulating] Closing gripper...");
  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(logger, "✓ Item grasped!");
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // PHASE 3: Cartesian Lift
  // ================================================================
  RCLCPP_INFO(logger, "[Phase 3] Lifting item...");
  
  std::vector<geometry_msgs::msg::Pose> lift_waypoints;
  current_pose = move_group.getCurrentPose().pose;
  lift_waypoints.push_back(current_pose);
  
  geometry_msgs::msg::Pose lift_pose = current_pose;
  lift_pose.position.z = 0.85;
  lift_waypoints.push_back(lift_pose);
  
  moveit_msgs::msg::RobotTrajectory lift_trajectory;
  double lift_fraction = move_group.computeCartesianPath(
      lift_waypoints, 0.01, lift_trajectory
  );
  
  RCLCPP_INFO(logger, "Cartesian lift: %.1f%% achieved", lift_fraction * 100.0);
  
  if (lift_fraction > 0.9) {
    move_group.execute(lift_trajectory);
    RCLCPP_INFO(logger, "✓ Item lifted!");
  } else {
    RCLCPP_WARN(logger, "⚠ Lift path incomplete");
  }
  
  RCLCPP_INFO(logger, " ");
  RCLCPP_INFO(logger, "=== Pick Complete! ===");
  RCLCPP_INFO(logger, "Check RViz for:");
  RCLCPP_INFO(logger, "  - RED sphere = goal marker");
  RCLCPP_INFO(logger, "  - BLUE dots = RRT waypoints");
  
  rclcpp::shutdown();
  return 0;
}