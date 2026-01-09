#!/usr/bin/env python3
"""
Add warehouse shelves and objects to MoveIt2 planning scene
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time


class WarehouseScenePublisher(Node):
    def __init__(self):
        super().__init__('warehouse_scene_publisher')
        
        # Publisher for planning scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        self.get_logger().info('Warehouse scene publisher ready')
        
        # Add warehouse objects
        self.add_warehouse_objects()
        
    def create_collision_object(self, name, frame_id, pose, dimensions):
        """
        Create a box collision object
        
        Args:
            name: Object ID
            frame_id: Reference frame (usually "panda_link0" or "world")
            pose: Pose (x, y, z, orientation)
            dimensions: [width, depth, height]
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = name
        
        # Define box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions  # [x, y, z]
        
        # Set pose
        box_pose = Pose()
        box_pose.position.x = pose[0]
        box_pose.position.y = pose[1]
        box_pose.position.z = pose[2]
        box_pose.orientation.w = 1.0  # No rotation
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object
    
    def add_warehouse_objects(self):
        """Add shelves, floor, and items to planning scene"""
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        
        frame_id = "panda_link0"  # Base frame for Panda arm
        
        # ----------------------------------------------------------
        # FLOOR (prevents arm from going below ground)
        # ----------------------------------------------------------
        floor = self.create_collision_object(
            name="floor",
            frame_id=frame_id,
            pose=[0.0, 0.0, -0.005],  # Just below origin
            dimensions=[2.0, 2.0, 0.01]  # Large flat plane
        )
        planning_scene.world.collision_objects.append(floor)
        self.get_logger().info('Added floor')
        
        # Move shelves FURTHER away
        shelf1 = self.create_collision_object(
            name="shelf_center",
            frame_id=frame_id,
            pose=[0.8, 0.0, 0.4],  # Changed from 0.7 to 0.8 (further away)
            dimensions=[0.6, 0.1, 0.8]
        )

        shelf2 = self.create_collision_object(
            name="shelf_left",
            frame_id=frame_id,
            pose=[0.8, 0.6, 0.4],  # Further apart
            dimensions=[0.6, 0.1, 0.8]
        )

        shelf3 = self.create_collision_object(
            name="shelf_right",
            frame_id=frame_id,
            pose=[0.8, -0.6, 0.4],  # Further apart
            dimensions=[0.6, 0.1, 0.8]
        )

        item = self.create_collision_object(
            name="target_item",
            frame_id=frame_id,
            pose=[0.8, 0.0, 0.85],  # Update item position too
            dimensions=[0.1, 0.1, 0.15]
        )
                
        # ----------------------------------------------------------
        # BACK WALL (Safety boundary)
        # ----------------------------------------------------------
        back_wall = self.create_collision_object(
            name="back_wall",
            frame_id=frame_id,
            pose=[0.9, 0.0, 0.5],  # Behind shelves
            dimensions=[0.01, 2.0, 1.0]  # Thin wall
        )
        planning_scene.world.collision_objects.append(back_wall)
        self.get_logger().info('Added back wall')
        
        # Publish the scene
        self.scene_pub.publish(planning_scene)
        self.get_logger().info('✓ Published warehouse scene with 5 objects')
        self.get_logger().info('  - Floor')
        self.get_logger().info('  - 3 Shelves (center, left, right)')
        self.get_logger().info('  - Target item at (0.8, 0.0, 0.85)')
        self.get_logger().info('  - Back wall')


def main(args=None):
    rclpy.init(args=args)
    
    node = WarehouseScenePublisher()
    
    # Keep node alive for a bit to ensure scene is published
    rclpy.spin_once(node, timeout_sec=2.0)
    
    node.get_logger().info('Warehouse scene setup complete!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
