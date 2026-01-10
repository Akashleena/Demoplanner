#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class WarehousePicker(Node):
    def __init__(self):
        super().__init__('warehouse_picker')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        time.sleep(1.0)
        self.setup_scene_and_start()
        
    def create_collision_object(self, name, frame_id, pose, dimensions):
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
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        frame_id = "panda_link0"
        
        # FIXED: Shelves at x=0.65 (WITHIN REACH!)
        floor = self.create_collision_object(
            "floor", frame_id, [0.0, 0.0, -0.005], [3.0, 3.0, 0.01]
        )
        shelf1 = self.create_collision_object(
            "shelf_center", frame_id, [0.65, 0.0, 0.3], [0.4, 0.05, 0.6]
        )
        shelf2 = self.create_collision_object(
            "shelf_left", frame_id, [0.65, 0.5, 0.3], [0.4, 0.05, 0.6]
        )
        shelf3 = self.create_collision_object(
            "shelf_right", frame_id, [0.65, -0.5, 0.3], [0.4, 0.05, 0.6]
        )
        item = self.create_collision_object(
            "target_item", frame_id, [0.65, 0.0, 0.70], [0.05, 0.05, 0.08]
        )
        back_wall = self.create_collision_object(
            "back_wall", frame_id, [1.0, 0.0, 0.5], [0.01, 3.0, 1.0]
        )
        
        planning_scene.world.collision_objects.extend([
            floor, shelf1, shelf2, shelf3, item, back_wall
        ])
        
        safe_joint_positions = [
            0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785
        ]
        
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        joint_state.position = safe_joint_positions
        
        planning_scene.robot_state.joint_state = joint_state
        planning_scene.robot_state.is_diff = False
        
        self.scene_pub.publish(planning_scene)
        
        self.get_logger().info('✓ Published warehouse scene')
        self.get_logger().info('✓ Shelves at x=0.65 (WITHIN REACH!)')
        self.get_logger().info('✓ Item at (0.65, 0.0, 0.70)')
        self.get_logger().info('✓ C++ will plan to (0.65, 0.0, 0.80) - above item')

def main(args=None):
    rclpy.init(args=args)
    node = WarehousePicker()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()