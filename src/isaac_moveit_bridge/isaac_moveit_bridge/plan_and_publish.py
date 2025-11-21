#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from moveit.planning import MoveItPy
import sys

class PlanAndPublish(Node):
    def __init__(self):
        super().__init__('plan_and_publish')
        self.get_logger().info("Initializing PlanAndPublish node...")

        # Initialize MoveItPy
        self.robot = MoveItPy(node_name="moveit_py", planning_groups=["arm_1"])
        self.arm_planning_component = self.robot.get_planning_component("arm_1")
        self.get_logger().info("MoveItPy planning component 'arm_1' initialized.")

        # Create a publisher for the planned trajectory
        self.trajectory_publisher = self.create_publisher(RobotTrajectory, '/planned_trajectory', 10)
        self.get_logger().info("Publisher for /planned_trajectory created.")

        # Give MoveItPy some time to initialize
        self.get_logger().info("Waiting for MoveItPy to be ready...")
        self.robot.wait_for_planning_pipeline()
        self.get_logger().info("MoveItPy is ready.")

        # Define a target pose for the end-effector (link_6)
        # You can modify this pose as needed
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0 # No rotation

        self.get_logger().info(f"Attempting to plan to target pose: {target_pose.pose}")

        # Plan to the target pose
        self.arm_planning_component.set_start_state_to_current_state()
        self.arm_planning_component.set_goal_from_pose(target_pose, "link_6") # "link_6" is the tip_link from SRDF

        plan_result = self.arm_planning_component.plan()

        if plan_result:
            self.get_logger().info("Planning successful! Publishing trajectory...")
            self.trajectory_publisher.publish(plan_result.trajectory)
            self.get_logger().info("Trajectory published. Shutting down node.")
        else:
            self.get_logger().error("Planning failed!")
        
        # Shutdown after planning and publishing
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PlanAndPublish()
    # The node destroys itself and shuts down rclpy within its __init__
    # so no need for rclpy.spin(node) here.

if __name__ == '__main__':
    main()
