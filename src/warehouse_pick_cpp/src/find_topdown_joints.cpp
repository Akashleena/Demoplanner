#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("find_topdown_joints");
    
    auto logger = rclcpp::get_logger("find_topdown");
    RCLCPP_INFO(logger, "Finding top-down start joints via IK...");
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "panda_arm");
    
    // Top-down pose
    geometry_msgs::msg::Pose top_down;
    top_down.position.x = 0.4;
    top_down.position.y = 0.0;
    top_down.position.z = 0.5;
    top_down.orientation.x = 1.0;  // 180° roll = top-down
    top_down.orientation.y = 0.0;
    top_down.orientation.z = 0.0;
    top_down.orientation.w = 0.0;
    
    move_group.setPoseTarget(top_down);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        auto& final_joints = plan.trajectory.joint_trajectory.points.back().positions;
        
        std::cout << "\n=== COPY THIS TO YOUR CODE ===" << std::endl;
        std::cout << "config.start_joints = {";
        for (size_t i = 0; i < final_joints.size(); ++i) {
            std::cout << final_joints[i];
            if (i < final_joints.size() - 1) std::cout << ", ";
        }
        std::cout << "};" << std::endl;
        std::cout << "==============================\n" << std::endl;
    } else {
        RCLCPP_ERROR(logger, "IK failed!");
    }
    
    rclcpp::shutdown();
    return 0;
}