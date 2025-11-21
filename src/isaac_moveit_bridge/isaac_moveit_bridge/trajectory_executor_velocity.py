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
    and controls the robot in Isaac Sim by sending a smooth stream of velocity commands.
    """
    def __init__(self, arm_joint_names, gripper_joint_names):
        super().__init__('moveit_to_isaac_bridge')
        self.arm_joint_names = arm_joint_names
        self.gripper_joint_names = gripper_joint_names
        self.full_robot_joint_names = self.arm_joint_names + self.gripper_joint_names

        self._current_joint_state = None
        self._state_lock = threading.Lock()
        self._trajectory_lock = threading.Lock() # New lock for trajectory execution

        # Declare and read parameters.
        self.declare_parameter('path_tolerance_rad', 100.0)
        self.declare_parameter('goal_tolerance_rad', 100.0)

        self._path_tolerance = self.get_parameter('path_tolerance_rad').get_parameter_value().double_value
        self._goal_tolerance = self.get_parameter('goal_tolerance_rad').get_parameter_value().double_value

        self.get_logger().info(f"Using Path Tolerance: {self._path_tolerance} rad")
        self.get_logger().info(f"Using Goal Tolerance: {self._goal_tolerance} rad")
        self.get_logger().info("Set tolerances to a large value (e.g., 100.0) in the launch file to disable checks.")

        self.command_pub = self.create_publisher(JointState, '/arm/joint_states_control', 10)
        self.state_sub = self.create_subscription(
            JointState, '/arm/joint_states', self.state_callback, 10)

        # New subscriber for planned trajectories from MoveIt
        self.display_trajectory_sub = self.create_subscription(
            DisplayTrajectory, '/display_planned_path', self.display_trajectory_callback, 10)
        self.get_logger().info("Subscribed to /display_planned_path for trajectory execution.")

        self.get_logger().info("MoveIt to Isaac Bridge (Subscriber Mode) is ready.")

    def state_callback(self, msg: JointState):
        with self._state_lock:
            self._current_joint_state = msg

    def display_trajectory_callback(self, msg: DisplayTrajectory):
        self.get_logger().info("Received new planned trajectory from MoveIt.")
        if not msg.trajectory_start.joint_state.name:
            self.get_logger().warn("Received an empty trajectory start state. Ignoring.")
            return

        if not msg.trajectory:
            self.get_logger().warn("Received an empty trajectory. Ignoring.")
            return

        # Assuming the first trajectory in the list is the one to execute
        trajectory = msg.trajectory[0]

        # Start execution in a new thread to avoid blocking the subscriber callback
        thread = threading.Thread(target=self._execute_trajectory, args=(trajectory,))
        thread.start()

    def _execute_trajectory(self, trajectory: RobotTrajectory):
        with self._trajectory_lock: # Ensure only one trajectory is executed at a time
            self.get_logger().info('Executing trajectory...')

            if not trajectory.joint_trajectory.points:
                self.get_logger().warn("Received a trajectory with no points. Skipping execution.")
                return

            control_rate = self.create_rate(100)
            start_time = self.get_clock().now()

            grace_period_duration = 0.5
            if len(trajectory.joint_trajectory.points) > 1:
                grace_period_duration = rclpy.duration.Duration.from_msg(trajectory.joint_trajectory.points[1].time_from_start).nanoseconds / 1e9

            while rclpy.ok():
                now = self.get_clock().now()
                elapsed_time = (now - start_time).nanoseconds / 1e9
                with self._state_lock:
                    if self._current_joint_state is None:
                        self.get_logger().warn("No robot state received yet. Waiting...")
                        control_rate.sleep()
                        continue
                    current_positions = np.array(self._current_joint_state.position)

                target_point = self.interpolate_trajectory(trajectory, elapsed_time)

                # New condition: break after planned duration + grace period
                planned_duration = rclpy.duration.Duration.from_msg(trajectory.joint_trajectory.points[-1].time_from_start).nanoseconds / 1e9
                post_trajectory_grace_period = 1.0 # seconds
                if elapsed_time > planned_duration + post_trajectory_grace_period:
                    self.get_logger().warn(f"Trajectory execution exceeded planned duration ({planned_duration:.3f}s) + grace period ({post_trajectory_grace_period:.1f}s). Stopping.")
                    break

                if elapsed_time > grace_period_duration:
                    error = np.linalg.norm(current_positions[0:6] - np.array(target_point.positions))
                    if error > self._path_tolerance:
                        self.stop_robot()
                        self.get_logger().error(f"Path deviation error ({error:.3f} rad) exceeds tolerance ({self._path_tolerance:.3f} rad). Stopping.")
                        return

                command_msg = JointState()
                command_msg.header.stamp = now.to_msg()
                command_msg.name = self.full_robot_joint_names
                arm_velocities = list(target_point.velocities) if target_point.velocities else [0.0] * len(self.arm_joint_names)
                gripper_velocities = [0.0] * len(self.gripper_joint_names)
                command_msg.velocity = arm_velocities + gripper_velocities
                self.command_pub.publish(command_msg)

                control_rate.sleep()

            self.stop_robot()
            final_goal_position = np.array(trajectory.joint_trajectory.points[-1].positions)
            with self._state_lock:
                if self._current_joint_state is None:
                    self.get_logger().error("Never received joint state. Cannot check final position.")
                    return
                final_actual_position = np.array(self._current_joint_state.position)

            final_error = np.linalg.norm(final_goal_position - final_actual_position[0:6])

            if final_error <= self._goal_tolerance:
                self.get_logger().info(f"Trajectory execution finished. Final error: {final_error:.3f} rad")
            else:
                self.get_logger().error(f"Trajectory execution finished with large error. Final error ({final_error:.3f} rad) exceeds tolerance ({self._goal_tolerance:.3f} rad).")

    def stop_robot(self):
        stop_msg = JointState()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.name = self.full_robot_joint_names
        stop_msg.velocity = [0.0] * len(self.full_robot_joint_names)
        self.command_pub.publish(stop_msg)
        self.get_logger().info("Sent stop command to robot.")

    def interpolate_trajectory(self, trajectory: RobotTrajectory, elapsed_time: float):
        if elapsed_time <= 0.0:
            return trajectory.joint_trajectory.points[0]

        last_point_time = rclpy.duration.Duration.from_msg(trajectory.joint_trajectory.points[-1].time_from_start).nanoseconds / 1e9
        if elapsed_time >= last_point_time:
            return trajectory.joint_trajectory.points[-1]

        for i in range(len(trajectory.joint_trajectory.points) - 1):
            p1 = trajectory.joint_trajectory.points[i]
            p2 = trajectory.joint_trajectory.points[i+1]
            t1 = rclpy.duration.Duration.from_msg(p1.time_from_start).nanoseconds / 1e9
            t2 = rclpy.duration.Duration.from_msg(p2.time_from_start).nanoseconds / 1e9

            if t1 <= elapsed_time < t2:
                alpha = (elapsed_time - t1) / (t2 - t1)
                interp_point = JointTrajectoryPoint()
                p1_pos = np.array(p1.positions)
                p2_pos = np.array(p2.positions)
                interp_point.positions = list(p1_pos + alpha * (p2_pos - p1_pos))

                if p1.velocities and p2.velocities:
                    p1_vel = np.array(p1.velocities)
                    p2_vel = np.array(p2.velocities)
                    interp_point.velocities = list(p1_vel + alpha * (p2_vel - p1_vel))

                interp_point.time_from_start = rclpy.duration.Duration(seconds=elapsed_time).to_msg()
                return interp_point

        return trajectory.joint_trajectory.points[-1]

def main(args=None):
    rclpy.init(args=args)

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
