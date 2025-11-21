#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_py.planning import MoveItPy
import random
import time

class RandomMovePlanner(Node):
    def __init__(self):
        super().__init__('random_move_planner')
        self.get_logger().info("Initializing RandomMovePlanner node...")

        # Initialize MoveItPy
        self.robot = MoveItPy(node_name="moveit_py_random_planner", planning_groups=["arm_1"])
        self.arm_planning_component = self.robot.get_planning_component("arm_1")
        self.get_logger().info("MoveItPy planning component 'arm_1' initialized.")

        # Give MoveItPy some time to initialize
        self.get_logger().info("Waiting for MoveItPy to be ready...")
        self.robot.wait_for_planning_pipeline()
        self.get_logger().info("MoveItPy is ready.")

        # Define workspace limits for random goal generation
        self.x_min, self.x_max = 0.2, 0.6
        self.y_min, self.y_max = -0.3, 0.3
        self.z_min, self.z_max = 0.2, 0.7

        self.timer = self.create_timer(5.0, self.plan_random_goal_callback) # Plan every 5 seconds

    def plan_random_goal_callback(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = random.uniform(self.x_min, self.x_max)
        target_pose.pose.position.y = random.uniform(self.y_min, self.y_max)
        target_pose.pose.position.z = random.uniform(self.z_min, self.z_max)
        target_pose.pose.orientation.w = 1.0 # No rotation for simplicity

        self.get_logger().info(f"Attempting to plan to random target pose: {target_pose.pose}")

        self.arm_planning_component.set_start_state_to_current_state()
        self.arm_planning_component.set_goal_from_pose(target_pose, "link_6")

        plan_result = self.arm_planning_component.plan()

        if plan_result:
            self.get_logger().info("Planning successful! MoveGroup will publish to /move_group/display_planned_path.")
            # The plan is automatically published by move_group to /move_group/display_planned_path
            # and trajectory_executor.py will pick it up.
        else:
            self.get_logger().error("Planning failed!")

def main(args=None):
    rclpy.init(args=args)
    node = RandomMovePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
