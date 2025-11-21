#!/usr/bin/env python3
"""
簡化版末端座標控制程式
使用 MoveGroup action interface (不需要 moveit_py)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import sys


class SimplifiedEndEffectorController(Node):
    """簡化版末端座標控制器"""
    
    def __init__(self):
        super().__init__('simplified_ee_controller')
        
        self.planning_group = 'arm_1'
        self.ee_link = 'j6_Link_L'
        self.base_frame = 'base_link'
        
        # MoveGroup Action Client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        self.get_logger().info('初始化完成!')
        self.get_logger().info(f'Planning Group: {self.planning_group}')
        self.get_logger().info(f'End Effector: {self.ee_link}')
    
    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """移動到指定的末端座標"""
        self.get_logger().info(f'收到目標座標: [{x:.3f}, {y:.3f}, {z:.3f}]')
        
        # 等待 action server
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup Action Server 未就緒')
            self.get_logger().error('請先啟動: ros2 launch arm_left_moveit_config arm_left_moveit.launch.py')
            return False
        
        # 建立 MoveGroup Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # 使用 cuMotion planner
        goal_msg.request.pipeline_id = 'isaac_ros_cumotion'
        goal_msg.request.planner_id = ''
        
        # 設定工作空間
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # 建立目標約束
        goal_constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.ee_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # 定義允許範圍
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # 1mm tolerance
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw
        
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        
        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0
        
        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # 發送 goal
        self.get_logger().info('發送規劃請求到 MoveGroup (使用 cuMotion)...')
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        
        # 等待接受
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        if not send_goal_future.done():
            self.get_logger().error('發送 goal 超時')
            return False
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('MoveGroup 拒絕規劃請求')
            return False
        
        self.get_logger().info('規劃請求已接受,等待結果...')
        
        # 等待結果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ 規劃並執行成功!')
            return True
        else:
            self.get_logger().error(f'✗ 失敗,錯誤碼: {result.error_code.val}')
            return False


def main():
    if len(sys.argv) < 4:
        print("使用方式:")
        print("  ros2 run arm_left_moveit_config simple_ee_control.py <x> <y> <z> [qx qy qz qw]")
        print("")
        print("範例:")
        print("  ros2 run arm_left_moveit_config simple_ee_control.py 0.3 0.0 0.5")
        print("")
        print("注意: 請先啟動 MoveIt:")
        print("  ros2 launch arm_left_moveit_config arm_left_moveit.launch.py")
        sys.exit(1)
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        
        if len(sys.argv) >= 8:
            qx = float(sys.argv[4])
            qy = float(sys.argv[5])
            qz = float(sys.argv[6])
            qw = float(sys.argv[7])
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    except ValueError:
        print("錯誤: 座標必須是數字")
        sys.exit(1)
    
    rclpy.init()
    controller = SimplifiedEndEffectorController()
    success = controller.move_to_pose(x, y, z, qx, qy, qz, qw)
    controller.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
