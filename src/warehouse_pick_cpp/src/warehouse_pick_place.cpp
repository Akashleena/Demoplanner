#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

// ============================================================================
// PICK-PLACE FRAME GENERATOR (Russ Tedrake style)
// ============================================================================

struct PickPlaceFrames {
    geometry_msgs::msg::Pose prepick;
    geometry_msgs::msg::Pose pick;
    geometry_msgs::msg::Pose postpick;
    geometry_msgs::msg::Pose clearance;
    geometry_msgs::msg::Pose preplace;
    geometry_msgs::msg::Pose place;
    geometry_msgs::msg::Pose postplace;
};

geometry_msgs::msg::Pose make_top_down_pose(double x, double y, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    // Top-down orientation (180° about X)
    pose.orientation.x = 1.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
}

PickPlaceFrames make_gripper_frames(
    double obj_x, double obj_y, double obj_z,    // object initial position
    double goal_x, double goal_y, double goal_z) // object goal position
{
    PickPlaceFrames frames;
    
    // Grasp parameters
    double grasp_height = 0.12;      // gripper height above object base
    double pregrasp_offset = 0.10;   // approach distance
    double clearance_height = 0.35;  // safe transit height
    
    // PICK poses
    frames.pick = make_top_down_pose(obj_x, obj_y, obj_z + grasp_height);
    frames.prepick = make_top_down_pose(obj_x, obj_y, obj_z + grasp_height + pregrasp_offset);
    frames.postpick = frames.prepick;  // retreat to same pose
    
    // PLACE poses
    frames.place = make_top_down_pose(goal_x, goal_y, goal_z + grasp_height);
    frames.preplace = make_top_down_pose(goal_x, goal_y, goal_z + grasp_height + pregrasp_offset);
    frames.postplace = frames.preplace;
    
    // CLEARANCE pose (midpoint, high up)
    double mid_x = (frames.prepick.position.x + frames.preplace.position.x) / 2.0;
    double mid_y = (frames.prepick.position.y + frames.preplace.position.y) / 2.0;
    frames.clearance = make_top_down_pose(mid_x, mid_y, clearance_height);
    
    return frames;
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "pick_place_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    auto logger = rclcpp::get_logger("pick_place");
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "panda_arm");
    
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(10.0);
    
    RCLCPP_INFO(logger, "=== Hybrid Pick-and-Place Demo ===");
    RCLCPP_INFO(logger, "");
    
    // ================================================================
    // PHASE 0: Go to home position
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 0] Moving to home...");
    std::vector<double> home_joints = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    move_group.setJointValueTarget(home_joints);
    move_group.move();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // GENERATE PICK-PLACE FRAMES
    // ================================================================
    // Object at pick location, goal at place location
    auto frames = make_gripper_frames(
        // 0.5, -0.25, 0.05,   // pick object position
        // 0.5,  0.25, 0.02    // place target position
        0.4, -0.2, 0.025,   // pick
        0.4,  0.2, 0.01     // place
);
    
    
    RCLCPP_INFO(logger, "Generated pick-place frames:");
    RCLCPP_INFO(logger, "  prepick:   (%.2f, %.2f, %.2f)", 
        frames.prepick.position.x, frames.prepick.position.y, frames.prepick.position.z);
    RCLCPP_INFO(logger, "  pick:      (%.2f, %.2f, %.2f)", 
        frames.pick.position.x, frames.pick.position.y, frames.pick.position.z);
    RCLCPP_INFO(logger, "  clearance: (%.2f, %.2f, %.2f)", 
        frames.clearance.position.x, frames.clearance.position.y, frames.clearance.position.z);
    RCLCPP_INFO(logger, "  preplace:  (%.2f, %.2f, %.2f)", 
        frames.preplace.position.x, frames.preplace.position.y, frames.preplace.position.z);
    RCLCPP_INFO(logger, "  place:     (%.2f, %.2f, %.2f)", 
        frames.place.position.x, frames.place.position.y, frames.place.position.z);
    RCLCPP_INFO(logger, "");
    
    // ================================================================
    // PHASE 1: RRT to prepick
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 1] RRT → prepick");
    move_group.setPoseTarget(frames.prepick, "panda_link8");
    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Failed to reach prepick!");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(logger, "✓ Reached prepick");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // PHASE 2: Cartesian approach to pick
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 2] Cartesian → pick (approach)");
    std::vector<geometry_msgs::msg::Pose> approach_waypoints = {frames.pick};
    moveit_msgs::msg::RobotTrajectory approach_traj;
    double approach_fraction = move_group.computeCartesianPath(
        approach_waypoints, 0.01, 0.0, approach_traj);
    
    if (approach_fraction > 0.9) {
        move_group.execute(approach_traj);
        RCLCPP_INFO(logger, "✓ Cartesian approach complete (%.0f%%)", approach_fraction * 100);
    } else {
        RCLCPP_WARN(logger, "Cartesian approach only %.0f%%, using RRT fallback", approach_fraction * 100);
        move_group.setPoseTarget(frames.pick, "panda_link8");
        move_group.move();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // PHASE 3: Grasp (simulated)
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 3] GRASP (gripper close)");
    rclcpp::sleep_for(std::chrono::seconds(1));  // Simulate gripper close
    RCLCPP_INFO(logger, "✓ Object grasped");
    // PHASE 3: Grasp - attach object to gripper

    // Attach object to end-effector
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "panda_link8";
    attached_object.object.id = "pick_object";
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    move_group.attachObject("pick_object", "panda_link8");

    RCLCPP_INFO(logger, "✓ Object attached to gripper");
    
    // ================================================================
    // PHASE 4: Cartesian retreat to postpick
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 4] Cartesian → postpick (retreat)");
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints = {frames.postpick};
    moveit_msgs::msg::RobotTrajectory retreat_traj;
    double retreat_fraction = move_group.computeCartesianPath(
        retreat_waypoints, 0.01, 0.0, retreat_traj);
    
    if (retreat_fraction > 0.9) {
        move_group.execute(retreat_traj);
        RCLCPP_INFO(logger, "✓ Cartesian retreat complete (%.0f%%)", retreat_fraction * 100);
    } else {
        RCLCPP_WARN(logger, "Cartesian retreat failed, using RRT");
        move_group.setPoseTarget(frames.postpick, "panda_link8");
        move_group.move();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // PHASE 5: RRT to clearance (over obstacle)
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 5] RRT → clearance (transit)");
    move_group.setPoseTarget(frames.clearance, "panda_link8");
    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Failed to reach clearance!");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(logger, "✓ Reached clearance");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // PHASE 6: RRT to preplace
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 6] RRT → preplace");
    move_group.setPoseTarget(frames.preplace, "panda_link8");
    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Failed to reach preplace!");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(logger, "✓ Reached preplace");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // PHASE 7: Cartesian approach to place
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 7] Cartesian → place (approach)");
    std::vector<geometry_msgs::msg::Pose> place_approach = {frames.place};
    moveit_msgs::msg::RobotTrajectory place_traj;
    double place_fraction = move_group.computeCartesianPath(
        place_approach, 0.01, 0.0, place_traj);
    
    if (place_fraction > 0.9) {
        move_group.execute(place_traj);
        RCLCPP_INFO(logger, "✓ Cartesian place approach complete (%.0f%%)", place_fraction * 100);
    } else {
        move_group.setPoseTarget(frames.place, "panda_link8");
        move_group.move();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ================================================================
    // PHASE 8: Release (simulated)
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 8] RELEASE (gripper open)");
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(logger, "✓ Object released");
    
    // ================================================================
    // PHASE 9: Cartesian retreat to postplace
    // ================================================================
    RCLCPP_INFO(logger, "[Phase 9] Cartesian → postplace (retreat)");
    std::vector<geometry_msgs::msg::Pose> final_retreat = {frames.postplace};
    moveit_msgs::msg::RobotTrajectory final_traj;
    double final_fraction = move_group.computeCartesianPath(
        final_retreat, 0.01, 0.0, final_traj);
    
    if (final_fraction > 0.9) {
        move_group.execute(final_traj);
    } else {
        move_group.setPoseTarget(frames.postplace, "panda_link8");
        move_group.move();
    }
    
    // ================================================================
    // DONE
    // ================================================================
    RCLCPP_INFO(logger, "");
    RCLCPP_INFO(logger, "=== Pick-and-Place Complete! ===");
    RCLCPP_INFO(logger, "Summary:");
    RCLCPP_INFO(logger, "  RRT:      home → prepick → clearance → preplace");
    RCLCPP_INFO(logger, "  Cartesian: prepick ↔ pick, preplace ↔ place");
    
    rclcpp::shutdown();
    return 0;
}