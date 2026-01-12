#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

// ============================================================================
// HELPER: Convert Quaternion to RPY and Print
// ============================================================================
void print_orientation_info(const geometry_msgs::msg::Quaternion& q, const std::string& label) {
    // Convert to tf2 quaternion
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    
    // Convert to RPY
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    std::cout << "\n=== " << label << " ===" << std::endl;
    std::cout << "Quaternion (x, y, z, w):" << std::endl;
    std::cout << "  x: " << q.x << std::endl;
    std::cout << "  y: " << q.y << std::endl;
    std::cout << "  z: " << q.z << std::endl;
    std::cout << "  w: " << q.w << std::endl;
    
    std::cout << "\nRoll-Pitch-Yaw (radians):" << std::endl;
    std::cout << "  Roll:  " << roll << " rad  (" << roll * 180.0 / M_PI << "°)" << std::endl;
    std::cout << "  Pitch: " << pitch << " rad  (" << pitch * 180.0 / M_PI << "°)" << std::endl;
    std::cout << "  Yaw:   " << yaw << " rad  (" << yaw * 180.0 / M_PI << "°)" << std::endl;
    
    // Generate C++ code snippet
    std::cout << "\nC++ Code to reproduce:" << std::endl;
    std::cout << "  goal_pose.orientation.x = " << q.x << ";" << std::endl;
    std::cout << "  goal_pose.orientation.y = " << q.y << ";" << std::endl;
    std::cout << "  goal_pose.orientation.z = " << q.z << ";" << std::endl;
    std::cout << "  goal_pose.orientation.w = " << q.w << ";" << std::endl;
    
    std::cout << "\nOr using RPY:" << std::endl;
    std::cout << "  tf2::Quaternion quat;" << std::endl;
    std::cout << "  quat.setRPY(" << roll << ", " << pitch << ", " << yaw << ");" << std::endl;
    std::cout << "  goal_pose.orientation = tf2::toMsg(quat);" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "orientation_finder",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    auto logger = rclcpp::get_logger("orientation_finder");
    
    RCLCPP_INFO(logger, "\n");
    RCLCPP_INFO(logger, "╔════════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(logger, "║   PANDA ROBOT ORIENTATION FINDER                              ║");
    RCLCPP_INFO(logger, "║   Find the correct vertical-down orientation for top grasping ║");
    RCLCPP_INFO(logger, "╚════════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(logger, "\n");
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "panda_arm");
    
    // ========================================================================
    // STEP 1: Get Current Orientation
    // ========================================================================
    RCLCPP_INFO(logger, "[STEP 1] Reading current end-effector pose...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    
    std::cout << "\nCurrent Position:" << std::endl;
    std::cout << "  x: " << current_pose.position.x << std::endl;
    std::cout << "  y: " << current_pose.position.y << std::endl;
    std::cout << "  z: " << current_pose.position.z << std::endl;
    
    print_orientation_info(current_pose.orientation, "CURRENT ORIENTATION (Home Position)");
    // ========================================================================
// STEP 1.5: Test specific joint configuration
// ========================================================================
RCLCPP_INFO(logger, "[STEP 1.5] Testing start joint configuration...");

std::vector<double> test_joints = {0.0, 0.5, 0.0, -1.5, 0.0, 2.0, 0.785};
move_group.setJointValueTarget(test_joints);

moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
if (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group.execute(joint_plan);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    print_orientation_info(start_pose.orientation, "START JOINTS ORIENTATION");
} else {
    RCLCPP_ERROR(logger, "Cannot reach test joints!");
}
    // ========================================================================
    // STEP 2: Test Common Vertical-Down Orientations
    // ========================================================================
    RCLCPP_INFO(logger, "\n[STEP 2] Testing common vertical-down orientations...\n");
    
    struct OrientationTest {
        std::string name;
        std::string description;
        double roll, pitch, yaw;  // In radians
    };
    
    std::vector<OrientationTest> tests = {
        {"Option 1", "Pure vertical down (180° roll)", M_PI, 0.0, 0.0},
        {"Option 2", "Vertical down (170° roll)", 170.0 * M_PI / 180.0, 0.0, 0.0},
        {"Option 3", "Vertical down (190° roll)", 190.0 * M_PI / 180.0, 0.0, 0.0},
        {"Option 4", "Vertical down (-180° roll)", -M_PI, 0.0, 0.0},
        {"Option 5", "90° pitch down", 0.0, M_PI/2.0, 0.0},
        {"Option 6", "90° pitch down + 180° roll", M_PI, M_PI/2.0, 0.0},
        {"Option 7", "-90° pitch down", 0.0, -M_PI/2.0, 0.0},
    };
    
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 0.40;  // Safe reachable position
    test_pose.position.y = 0.0;
    test_pose.position.z = 0.50;
    
    RCLCPP_INFO(logger, "Test position: (%.2f, %.2f, %.2f)\n", 
                test_pose.position.x, test_pose.position.y, test_pose.position.z);
    
    std::vector<int> successful_tests;
    
    for (size_t i = 0; i < tests.size(); ++i) {
        const auto& test = tests[i];
        
        // Convert RPY to quaternion
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(test.roll, test.pitch, test.yaw);
        test_pose.orientation = tf2::toMsg(quat_tf);
        
        std::cout << "─────────────────────────────────────────" << std::endl;
        std::cout << "Testing " << test.name << ": " << test.description << std::endl;
        std::cout << "  Roll:  " << test.roll * 180.0 / M_PI << "°" << std::endl;
        std::cout << "  Pitch: " << test.pitch * 180.0 / M_PI << "°" << std::endl;
        std::cout << "  Yaw:   " << test.yaw * 180.0 / M_PI << "°" << std::endl;
        
        // Try planning
        move_group.setPoseTarget(test_pose);
        move_group.setPlanningTime(5.0);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group.plan(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            std::cout << "✓✓✓ SUCCESS! This orientation is REACHABLE! ✓✓✓" << std::endl;
            successful_tests.push_back(i);
            print_orientation_info(test_pose.orientation, test.name + " - VERIFIED");
        } else {
            std::cout << "✗ FAILED - Cannot reach this orientation" << std::endl;
        }
        std::cout << std::endl;
    }
    
    // ========================================================================
    // STEP 3: Interactive Mode - Wait for Manual RViz Adjustment
    // ========================================================================
    RCLCPP_INFO(logger, "\n[STEP 3] Interactive RViz Mode");
    RCLCPP_INFO(logger, "═════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger, "\nINSTRUCTIONS:");
    RCLCPP_INFO(logger, "1. In RViz, enable 'MotionPlanning' -> 'Query' -> 'Interactive Markers'");
    RCLCPP_INFO(logger, "2. Drag and rotate the end-effector to VERTICAL DOWN position");
    RCLCPP_INFO(logger, "3. Once positioned correctly, run this command in a new terminal:");
    RCLCPP_INFO(logger, "     ros2 topic echo /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update");
    RCLCPP_INFO(logger, "4. Copy the orientation quaternion values from the topic output");
    RCLCPP_INFO(logger, "\nPress Ctrl+C when done.\n");
    
    // ========================================================================
    // STEP 4: Summary
    // ========================================================================
    if (successful_tests.empty()) {
        RCLCPP_WARN(logger, "\n⚠ No standard orientations were reachable at test position.");
        RCLCPP_WARN(logger, "Try the Interactive RViz method above!");
    } else {
        RCLCPP_INFO(logger, "\n╔═══════════════════════════════════════╗");
        RCLCPP_INFO(logger, "║  SUCCESSFUL VERTICAL-DOWN ORIENTATIONS  ║");
        RCLCPP_INFO(logger, "╚═══════════════════════════════════════╝\n");
        
        for (int idx : successful_tests) {
            const auto& test = tests[idx];
            
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(test.roll, test.pitch, test.yaw);
            geometry_msgs::msg::Quaternion q = tf2::toMsg(quat_tf);
            
            std::cout << test.name << " - " << test.description << std::endl;
            std::cout << "  goal_pose.orientation.x = " << q.x << ";" << std::endl;
            std::cout << "  goal_pose.orientation.y = " << q.y << ";" << std::endl;
            std::cout << "  goal_pose.orientation.z = " << q.z << ";" << std::endl;
            std::cout << "  goal_pose.orientation.w = " << q.w << ";\n" << std::endl;
        }
        
        RCLCPP_INFO(logger, "Copy the code above into your pick-and-place application!");
    }
    
    // Keep node alive for RViz interaction
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}