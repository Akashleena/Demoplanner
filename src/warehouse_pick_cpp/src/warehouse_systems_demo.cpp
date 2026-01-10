#include <memory>
#include <vector>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

// ============================================================================
// STAGE 2: Trajectory Scoring
// ============================================================================
struct TrajectoryScore {
    bool valid;
    bool dense_collision_ok;  // Stage 4 (will implement later)
    double path_length;
    double smoothness;
    double total_cost;
    
    TrajectoryScore() 
        : valid(false), dense_collision_ok(true), 
          path_length(0.0), smoothness(0.0), total_cost(std::numeric_limits<double>::max()) {}
};

double compute_path_length(const robot_trajectory::RobotTrajectory& traj) {
    double length = 0.0;
    const auto& joint_model_group = traj.getGroup();
    
    for (size_t i = 1; i < traj.getWayPointCount(); ++i) {
        const auto& prev = traj.getWayPoint(i-1);
        const auto& curr = traj.getWayPoint(i);
        
        std::vector<double> prev_joints, curr_joints;
        prev.copyJointGroupPositions(joint_model_group, prev_joints);
        curr.copyJointGroupPositions(joint_model_group, curr_joints);
        
        double segment_dist = 0.0;
        for (size_t j = 0; j < prev_joints.size(); ++j) {
            double delta = curr_joints[j] - prev_joints[j];
            segment_dist += delta * delta;
        }
        length += std::sqrt(segment_dist);
    }
    return length;
}

double compute_smoothness(const robot_trajectory::RobotTrajectory& traj) {
    if (traj.getWayPointCount() < 3) return 0.0;
    
    double smoothness = 0.0;
    const auto& joint_model_group = traj.getGroup();
    
    for (size_t i = 1; i < traj.getWayPointCount() - 1; ++i) {
        const auto& prev = traj.getWayPoint(i-1);
        const auto& curr = traj.getWayPoint(i);
        const auto& next = traj.getWayPoint(i+1);
        
        std::vector<double> prev_joints, curr_joints, next_joints;
        prev.copyJointGroupPositions(joint_model_group, prev_joints);
        curr.copyJointGroupPositions(joint_model_group, curr_joints);
        next.copyJointGroupPositions(joint_model_group, next_joints);
        
        // Second difference (curvature measure)
        for (size_t j = 0; j < curr_joints.size(); ++j) {
            double second_diff = next_joints[j] - 2*curr_joints[j] + prev_joints[j];
            smoothness += second_diff * second_diff;
        }
    }
    return smoothness;
}

TrajectoryScore score_trajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    TrajectoryScore score;
    score.valid = true;  // Plan succeeded
    
    // Access joint trajectory (FIXED: removed trailing underscore for Jazzy)
    const auto& joint_traj = plan.trajectory.joint_trajectory;
    
    // Path length: sum of squared joint deltas
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
    
    // Smoothness: sum of squared second differences
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
    
    // Total cost: weighted sum
    const double alpha = 1.0;  // path length weight
    const double beta = 0.5;   // smoothness weight
    score.total_cost = alpha * score.path_length + beta * score.smoothness;
    
    // Stage 4 penalty (placeholder for now)
    if (!score.dense_collision_ok) {
        score.total_cost += 1000.0;  // Large penalty
    }
    
    return score;
}

// ============================================================================
// MAIN: MVP Systems Demo
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
  
  RCLCPP_INFO(logger, "=== Warehouse Systems Demo ===");
  RCLCPP_INFO(logger, "Stages 1-2: Multi-start Planning + Scoring");
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
  // STAGE 1: Multi-Start Planning (K=6)
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 1] Multi-start planning (K=6)...");
  RCLCPP_INFO(logger, " ");
  
  // Goal: Safe approach pose (AWAY from shelf, reachable)
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 0.6;   // 60cm from robot base
  goal_pose.position.y = 0.0;   
  goal_pose.position.z = 0.5;
  goal_pose.orientation.w = 1.0;
  
  move_group.setPoseTarget(goal_pose);
  move_group.setPlanningTime(5.0);
  
  const int K = 6;  // Number of planning attempts
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> candidates;
  std::vector<TrajectoryScore> scores;
  
  // Sequential planning attempts
  for (int i = 0; i < K; ++i) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group.plan(plan);
    
    TrajectoryScore score;
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      score = score_trajectory(plan);
      candidates.push_back(plan);
      
      RCLCPP_INFO(logger, "Attempt %d | valid=1 | dense=1 | score=%.2f | len=%.2f | smooth=%.2f",
                  i, score.total_cost, score.path_length, score.smoothness);
    } else {
      score.valid = false;
      RCLCPP_INFO(logger, "Attempt %d | valid=0", i);
    }
    
    scores.push_back(score);
  }
  
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // STAGE 2: Select Best Candidate
  // ================================================================
  RCLCPP_INFO(logger, "[Stage 2] Selecting best candidate...");
  
  if (candidates.empty()) {
    RCLCPP_ERROR(logger, "✗ No valid plans found!");
    rclcpp::shutdown();
    return 1;
  }
  
  // Find best among valid attempts
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
      best_idx = i;  // Index into candidates array
    }
  }
  
  RCLCPP_INFO(logger, "Best attempt: %d (cost=%.2f)", valid_indices[best_idx], best_cost);
  RCLCPP_INFO(logger, " ");
  
  // ================================================================
  // Execute Best Plan
  // ================================================================
  RCLCPP_INFO(logger, "Executing best trajectory...");
  
  if (best_idx >= 0) {
    move_group.execute(candidates[best_idx]);
    RCLCPP_INFO(logger, "✓ Execution complete");
  }
  
  RCLCPP_INFO(logger, " ");
  RCLCPP_INFO(logger, "=== Demo Complete (Stages 1-2) ===");
  RCLCPP_INFO(logger, "Next: Implement Stage 3 (CHOMP-lite smoothing)");
  
  rclcpp::shutdown();
  return 0;
}