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
        
        pick_object = self.create_collision_object(
            "pick_object", frame_id, [0.4, -0.2, 0.025], [0.05, 0.05, 0.05]
        )
        
        place_marker = self.create_collision_object(
            "place_marker", frame_id, [0.4, 0.2, 0.01], [0.06, 0.06, 0.01]
        )
        
        planning_scene.world.collision_objects.extend([
            pick_object, place_marker
        ])
        
        # PUBLISH THE SCENE!
        self.scene_pub.publish(planning_scene)
        self.get_logger().info('✓ Published pick-place scene')

def main(args=None):
    rclpy.init(args=args)
    node = WarehousePicker()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()