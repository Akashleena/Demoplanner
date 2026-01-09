#!/usr/bin/env python3
"""
Plan pick motion in warehouse with custom start state
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


class WarehousePicker(Node):
    def __init__(self):
        super().__init__('warehouse_picker')
        
        # Publisher for planning scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        time.sleep(1.0)
        
        # Add warehouse and set safe start
        self.setup_scene_and_start()
        
    def create_collision_object(self, name, frame_id, pose, dimensions):
        """Create a box collision object"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = name
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        
        box_pose = Pose()
        box_pose.position.x = pose[0]
        box_pose.position.y = pose[1]
        box_pose.position.z = pose[2]
        box_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object
    
    def setup_scene_and_start(self):
        """Add warehouse objects AND set safe start configuration"""
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        frame_id = "panda_link0"
        
        # Add all warehouse objects (same as before)
        floor = self.create_collision_object(
            "floor", frame_id, [0.0, 0.0, -0.005], [2.0, 2.0, 0.01]
        )
        shelf1 = self.create_collision_object(
            "shelf_center", frame_id, [0.7, 0.0, 0.4], [0.6, 0.1, 0.8]
        )
        shelf2 = self.create_collision_object(
            "shelf_left", frame_id, [0.7, 0.5, 0.4], [0.6, 0.1, 0.8]
        )
        # shelf3 = self.create_collision_object(
        #     "shelf_right", frame_id, [0.7, -0.5, 0.4], [0.6, 0.1, 0.8]
        # )
        shelf3 = self.create_collision_object(
        "shelf_right", frame_id, [0.7, -0.6, 0.4], [0.6, 0.1, 0.8]  # Changed y from -0.5 to -0.6
        )
        item = self.create_collision_object(
            "target_item", frame_id, [0.7, 0.0, 0.85], [0.1, 0.1, 0.15]
        )
        back_wall = self.create_collision_object(
            "back_wall", frame_id, [1.1, 0.0, 0.5], [0.01, 2.0, 1.0]
        )
        
        planning_scene.world.collision_objects.extend([
            floor, shelf1, shelf2, shelf3, item, back_wall
        ])
        
        # ----------------------------------------------------------
        # SET SAFE START CONFIGURATION
        # ----------------------------------------------------------
        # Panda "ready" pose - arm tucked, away from shelves
        safe_joint_positions = [
            0.0,      # panda_joint1
            -0.785,   # panda_joint2 (elbow up)
            0.0,      # panda_joint3
            -2.356,   # panda_joint4 (elbow bent)
            0.0,      # panda_joint5
            1.571,    # panda_joint6
            0.785,    # panda_joint7
        ]
        
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        joint_state.position = safe_joint_positions
        
        planning_scene.robot_state.joint_state = joint_state
        planning_scene.robot_state.is_diff = False  # Set absolute state
        
        # Publish scene with new start state
        self.scene_pub.publish(planning_scene)
        
        self.get_logger().info('✓ Published warehouse scene')
        self.get_logger().info('✓ Set robot to SAFE START configuration')
        self.get_logger().info('  Joint positions: ' + str(safe_joint_positions))
        self.get_logger().info('')
        self.get_logger().info('Now you can plan to target at (0.7, 0.0, 0.85)')


def main(args=None):
    rclpy.init(args=args)
    node = WarehousePicker()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()