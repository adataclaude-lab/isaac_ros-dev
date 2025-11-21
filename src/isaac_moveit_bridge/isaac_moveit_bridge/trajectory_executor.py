#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointStateRepublisher(Node):
    """
    This node subscribes to the raw joint states from Isaac Sim (including gripper),
    filters out the gripper joints, and republishes the arm-only joint states for MoveIt.
    """
    def __init__(self, arm_joint_names):
        super().__init__('joint_state_republisher')
        self.arm_joint_names = arm_joint_names
        self.filtered_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.raw_sub = self.create_subscription(
            JointState, '/arm/joint_states', self.raw_state_callback, 10)
        self.get_logger().info("Joint State Republisher is ready.")

    def raw_state_callback(self, msg: JointState):
        try:
            name_to_index_map = {name: i for i, name in enumerate(msg.name)}
            filtered_msg = JointState()
            filtered_msg.header.stamp = self.get_clock().now().to_msg()
            filtered_msg.name = self.arm_joint_names
            filtered_msg.position = [msg.position[name_to_index_map[name]] for name in self.arm_joint_names]
            filtered_msg.velocity = [msg.velocity[name_to_index_map[name]] for name in self.arm_joint_names]
            if msg.effort:
                filtered_msg.effort = [msg.effort[name_to_index_map[name]] for name in self.arm_joint_names]
            self.filtered_pub.publish(filtered_msg)
        except (KeyError, IndexError) as e:
            self.get_logger().warn(f"Error processing joint states from Isaac Sim: {e}. Check joint names.")

class TrajectorySubscriber(Node):
    """
    This node subscribes to planned trajectories from MoveIt's /move_group/display_planned_path topic
    and controls the robot in Isaac Sim by sending a stream of position commands.
    """
    def __init__(self, arm_joint_names, gripper_joint_names):
        super().__init__('moveit_to_isaac_bridge')
        self.arm_joint_names = arm_joint_names
        self.gripper_joint_names = gripper_joint_names
        self.full_robot_joint_names = self.arm_joint_names + self.gripper_joint_names

        self._current_joint_state = None
        self._state_lock = threading.Lock()
        self._trajectory_lock = threading.Lock() # Lock to prevent concurrent executions

        # Declare and read parameters.
        self.declare_parameter('path_tolerance_rad', 100.0)
        self.declare_parameter('goal_tolerance_rad', 100.0)
        self.declare_parameter('position_tolerance_rad', 0.05) # 0.05 radians = ~2.8 degrees

        self._path_tolerance = self.get_parameter('path_tolerance_rad').get_parameter_value().double_value
        self._goal_tolerance = self.get_parameter('goal_tolerance_rad').get_parameter_value().double_value
        self._position_tolerance = self.get_parameter('position_tolerance_rad').get_parameter_value().double_value

        self.get_logger().info(f"Using Path Tolerance: {self._path_tolerance} rad")
        self.get_logger().info(f"Using Goal Tolerance: {self._goal_tolerance} rad")
        self.get_logger().info(f"Using Position Tolerance for waypoints: {self._position_tolerance} rad")

        self.command_pub = self.create_publisher(JointState, '/arm/joint_states_control', 10)
        self.state_sub = self.create_subscription(
            JointState, '/arm/joint_states', self.state_callback, 10)

        # Subscriber for planned trajectories from MoveIt
        self.display_trajectory_sub = self.create_subscription(
            DisplayTrajectory, '/display_planned_path', self.display_trajectory_callback, 10)
        self.get_logger().info("Subscribed to /display_planned_path for trajectory execution.")

        self.get_logger().info("MoveIt to Isaac Bridge (Subscriber Mode) is ready.")

    def state_callback(self, msg: JointState):
        with self._state_lock:
            self._current_joint_state = msg

    def display_trajectory_callback(self, msg: DisplayTrajectory):
        self.get_logger().info("Received new planned trajectory from MoveIt.")
        
        # Try to acquire the lock in a non-blocking way
        if not self._trajectory_lock.acquire(blocking=False):
            self.get_logger().warn("Trajectory execution in progress. Ignoring new trajectory.")
            return  # Exit the callback; do not start a new thread

        # If we are here, we successfully acquired the lock.
        # We will pass the responsibility of releasing it to the thread.

        if not msg.trajectory_start.joint_state.name:
            self.get_logger().warn("Received an empty trajectory start state. Ignoring.")
            self._trajectory_lock.release()  # Release lock if we exit early
            return

        if not msg.trajectory:
            self.get_logger().warn("Received an empty trajectory. Ignoring.")
            self._trajectory_lock.release()  # Release lock if we exit early
            return

        # Assuming the first trajectory in the list is the one to execute
        trajectory = msg.trajectory[0]

        # Start execution in a new thread to avoid blocking the subscriber callback
        # This thread is now responsible for releasing the lock
        thread = threading.Thread(target=self._execute_trajectory, args=(trajectory,))
        thread.start()

    def _execute_trajectory(self, trajectory: RobotTrajectory):
        # The lock was acquired in the callback.
        # We MUST release it when this function exits, no matter what.
        try:
            self.get_logger().info('Executing trajectory with position control...')

            if not trajectory.joint_trajectory.points:
                self.get_logger().warn("Received a trajectory with no points. Skipping execution.")
                return # The 'finally' block will still run

            # Use the position tolerance value stored from __init__
            self.get_logger().info(f"Using Position Tolerance for waypoints: {self._position_tolerance} rad")

            control_rate = self.create_rate(100) # Still send commands at 100 Hz

            for i, target_point in enumerate(trajectory.joint_trajectory.points):
                self.get_logger().info(f"Moving to waypoint {i+1}/{len(trajectory.joint_trajectory.points)}...")

                # Publish target position
                command_msg = JointState()
                command_msg.header.stamp = self.get_clock().now().to_msg()
                command_msg.name = self.full_robot_joint_names
                # Assuming Isaac Sim accepts position commands on this topic
                arm_positions = list(target_point.positions)
                gripper_positions = [0.0] * len(self.gripper_joint_names) # Keep grippers at 0 position
                command_msg.position = arm_positions + gripper_positions
                # Clear velocities and efforts to ensure position control
                command_msg.velocity = []
                command_msg.effort = []
                self.command_pub.publish(command_msg)

                # Wait until current position is close to target position
                start_wait_time = self.get_clock().now()
                timeout_duration = 5.0 # seconds, to prevent infinite waiting
                
                while rclpy.ok():
                    current_positions = None
                    with self._state_lock:
                        if self._current_joint_state is None:
                            self.get_logger().warn("No robot state received yet. Waiting for state...")
                            control_rate.sleep()
                            continue
                        current_positions = np.array(self._current_joint_state.position)

                    # Check if reached target position
                    # Ensure we only compare the arm joints
                    current_arm_positions = current_positions[:len(self.arm_joint_names)]
                    target_arm_positions = np.array(target_point.positions)
                    
                    position_error = np.linalg.norm(current_arm_positions - target_arm_positions)

                    # Use the class member self._position_tolerance
                    if position_error < self._position_tolerance:
                        self.get_logger().info(f"Waypoint {i+1} reached. Error: {position_error:.3f} rad")
                        break # Move to next waypoint

                    # Check for timeout
                    elapsed_wait_time = (self.get_clock().now() - start_wait_time).nanoseconds / 1e9
                    if elapsed_wait_time > timeout_duration:
                        self.get_logger().warn(f"Timeout waiting for waypoint {i+1}. Current error: {position_error:.3f} rad. Moving to next waypoint.")
                        break # Move to next waypoint even if not perfectly reached

                    control_rate.sleep()

            self.stop_robot() # Stop robot after all waypoints are processed

            # Final error check
            final_goal_position = np.array(trajectory.joint_trajectory.points[-1].positions)
            final_actual_position = None
            with self._state_lock:
                if self._current_joint_state is None:
                    self.get_logger().error("Never received joint state. Cannot check final position.")
                    return # The 'finally' block will still run
                final_actual_position = np.array(self._current_joint_state.position[:len(self.arm_joint_names)]) # Only arm joints

            final_error = np.linalg.norm(final_goal_position - final_actual_position)

            if final_error <= self._goal_tolerance: # Use the existing goal tolerance
                self.get_logger().info(f"Trajectory execution finished. Final error: {final_error:.3f} rad")
            else:
                self.get_logger().error(f"Trajectory execution finished with large error. Final error ({final_error:.3f} rad) exceeds tolerance ({self._goal_tolerance:.3f} rad).")
        
        finally:
            self.get_logger().info("Trajectory execution finished. Releasing lock.")
            self._trajectory_lock.release() # Ensure lock is always released

    def stop_robot(self):
        # This function might be better if it sent the *current* position
        # as the position command, and zero velocity, but zero velocity
        # on its own might not be interpreted as "stop" by a position controller.
        # For a velocity controller, this is correct.
        # For a position controller (like Isaac Sim often uses), 
        # it's better to just *stop sending* commands, or send the last known position.
        # However, sending a zero-velocity JointState is harmless.
        stop_msg = JointState()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.name = self.full_robot_joint_names
        stop_msg.velocity = [0.0] * len(self.full_robot_joint_names)
        self.command_pub.publish(stop_msg)
        self.get_logger().info("Sent stop (zero velocity) command to robot.")

def main(args=None):
    rclpy.init(args=args)

    # Define joint names
    arm_joints = [f'arm_1_joint_{i}' for i in range(1, 7)]
    gripper_joints = ['joint_7', 'joint_8']

    republisher = JointStateRepublisher(arm_joint_names=arm_joints)
    trajectory_executor = TrajectorySubscriber(arm_joint_names=arm_joints, gripper_joint_names=gripper_joints)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(republisher)
    executor.add_node(trajectory_executor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        republisher.destroy_node()
        trajectory_executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()