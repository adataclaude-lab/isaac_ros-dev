#!/usr/bin/env python3
"""
雙手臂末端執行器控制腳本
支援左右手臂分別控制或同時控制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionIK
import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

try:
    from moveit_py import MoveItPy, PlanningComponent
    from moveit_py.robot_state import RobotState as MoveItRobotState
except ImportError:
    print("Error: moveit_py not found. Please install moveit2 python bindings.")
    sys.exit(1)


class DualArmController(Node):
    """雙手臂控制器 - 支援左右手臂獨立或同時控制"""

    def __init__(self):
        super().__init__('dual_arm_controller')

        self.get_logger().info('初始化雙手臂控制器...')

        # 創建回調組以支持並行操作
        self.callback_group = ReentrantCallbackGroup()

        # 初始化 MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py_dual_arm")

        # 獲取左右手臂規劃組件
        self.left_arm = self.moveit.get_planning_component("left_arm")
        self.right_arm = self.moveit.get_planning_component("right_arm")

        # 獲取機器人模型
        self.robot_model = self.moveit.get_robot_model()

        # 獲取規劃場景
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        self.get_logger().info('雙手臂控制器初始化完成')
        self.get_logger().info('左手臂規劃組: left_arm')
        self.get_logger().info('右手臂規劃組: right_arm')

    def move_left_arm_to_pose(self, target_pose):
        """
        移動左手臂到目標位姿

        Args:
            target_pose: geometry_msgs/Pose - 目標位姿

        Returns:
            bool: 是否成功
        """
        self.get_logger().info('規劃左手臂運動...')

        # 設置目標位姿
        self.left_arm.set_goal_state(
            pose_stamped_msg=self._create_pose_stamped(target_pose, "Dual_arm_Robot_Entire"),
            pose_link="j6_Link_L"
        )

        # 規劃
        plan_result = self.left_arm.plan()

        if plan_result:
            self.get_logger().info('左手臂規劃成功，執行運動...')
            # 執行
            success = self.left_arm.execute(blocking=True)
            if success:
                self.get_logger().info('左手臂運動執行成功')
            else:
                self.get_logger().error('左手臂運動執行失敗')
            return success
        else:
            self.get_logger().error('左手臂規劃失敗')
            return False

    def move_right_arm_to_pose(self, target_pose):
        """
        移動右手臂到目標位姿

        Args:
            target_pose: geometry_msgs/Pose - 目標位姿

        Returns:
            bool: 是否成功
        """
        self.get_logger().info('規劃右手臂運動...')

        # 設置目標位姿
        self.right_arm.set_goal_state(
            pose_stamped_msg=self._create_pose_stamped(target_pose, "Dual_arm_Robot_Entire"),
            pose_link="j6_Link_R"
        )

        # 規劃
        plan_result = self.right_arm.plan()

        if plan_result:
            self.get_logger().info('右手臂規劃成功，執行運動...')
            # 執行
            success = self.right_arm.execute(blocking=True)
            if success:
                self.get_logger().info('右手臂運動執行成功')
            else:
                self.get_logger().error('右手臂運動執行失敗')
            return success
        else:
            self.get_logger().error('右手臂規劃失敗')
            return False

    def move_both_arms_to_pose(self, left_pose, right_pose):
        """
        同時移動左右手臂到目標位姿

        Args:
            left_pose: geometry_msgs/Pose - 左手臂目標位姿
            right_pose: geometry_msgs/Pose - 右手臂目標位姿

        Returns:
            bool: 是否成功
        """
        self.get_logger().info('規劃雙手臂協調運動...')

        # 設置左手臂目標
        self.left_arm.set_goal_state(
            pose_stamped_msg=self._create_pose_stamped(left_pose, "Dual_arm_Robot_Entire"),
            pose_link="j6_Link_L"
        )

        # 設置右手臂目標
        self.right_arm.set_goal_state(
            pose_stamped_msg=self._create_pose_stamped(right_pose, "Dual_arm_Robot_Entire"),
            pose_link="j6_Link_R"
        )

        # 規劃左手臂
        left_plan = self.left_arm.plan()

        # 規劃右手臂
        right_plan = self.right_arm.plan()

        if left_plan and right_plan:
            self.get_logger().info('雙手臂規劃成功，執行協調運動...')

            # 先執行左手臂
            left_success = self.left_arm.execute(blocking=True)

            # 再執行右手臂
            right_success = self.right_arm.execute(blocking=True)

            success = left_success and right_success

            if success:
                self.get_logger().info('雙手臂協調運動執行成功')
            else:
                self.get_logger().error('雙手臂協調運動執行失敗')

            return success
        else:
            if not left_plan:
                self.get_logger().error('左手臂規劃失敗')
            if not right_plan:
                self.get_logger().error('右手臂規劃失敗')
            return False

    def move_left_arm_to_joints(self, joint_values):
        """
        移動左手臂到關節角度

        Args:
            joint_values: list - 6個關節角度值

        Returns:
            bool: 是否成功
        """
        if len(joint_values) != 6:
            self.get_logger().error('需要6個關節角度值')
            return False

        self.get_logger().info(f'規劃左手臂關節運動: {joint_values}')

        # 設置關節目標
        joint_dict = {
            'j1_Joint_L': joint_values[0],
            'j2_Joint_L': joint_values[1],
            'j3_Joint_L': joint_values[2],
            'j4_Joint_L': joint_values[3],
            'j5_Joint_L': joint_values[4],
            'j6_Joint_L': joint_values[5]
        }

        self.left_arm.set_goal_state(configuration_name="")
        robot_state = self.left_arm.get_start_state()

        for joint_name, value in joint_dict.items():
            robot_state.joint_positions[joint_name] = [value]

        self.left_arm.set_goal_state(robot_state=robot_state)

        # 規劃並執行
        plan_result = self.left_arm.plan()

        if plan_result:
            self.get_logger().info('左手臂關節規劃成功，執行運動...')
            success = self.left_arm.execute(blocking=True)
            if success:
                self.get_logger().info('左手臂關節運動執行成功')
            else:
                self.get_logger().error('左手臂關節運動執行失敗')
            return success
        else:
            self.get_logger().error('左手臂關節規劃失敗')
            return False

    def move_right_arm_to_joints(self, joint_values):
        """
        移動右手臂到關節角度

        Args:
            joint_values: list - 6個關節角度值

        Returns:
            bool: 是否成功
        """
        if len(joint_values) != 6:
            self.get_logger().error('需要6個關節角度值')
            return False

        self.get_logger().info(f'規劃右手臂關節運動: {joint_values}')

        # 設置關節目標
        joint_dict = {
            'j1_Joint_R': joint_values[0],
            'j2_Joint_R': joint_values[1],
            'j3_Joint_R': joint_values[2],
            'j4_Joint_R': joint_values[3],
            'j5_Joint_R': joint_values[4],
            'j6_Joint_R': joint_values[5]
        }

        self.right_arm.set_goal_state(configuration_name="")
        robot_state = self.right_arm.get_start_state()

        for joint_name, value in joint_dict.items():
            robot_state.joint_positions[joint_name] = [value]

        self.right_arm.set_goal_state(robot_state=robot_state)

        # 規劃並執行
        plan_result = self.right_arm.plan()

        if plan_result:
            self.get_logger().info('右手臂關節規劃成功，執行運動...')
            success = self.right_arm.execute(blocking=True)
            if success:
                self.get_logger().info('右手臂關節運動執行成功')
            else:
                self.get_logger().error('右手臂關節運動執行失敗')
            return success
        else:
            self.get_logger().error('右手臂關節規劃失敗')
            return False

    def move_to_initial_pose(self):
        """移動到初始位姿"""
        self.get_logger().info('移動到初始位姿...')

        # 設置左手臂初始位姿
        self.left_arm.set_goal_state(configuration_name="initial")
        left_plan = self.left_arm.plan()

        # 設置右手臂初始位姿
        self.right_arm.set_goal_state(configuration_name="initial")
        right_plan = self.right_arm.plan()

        if left_plan and right_plan:
            left_success = self.left_arm.execute(blocking=True)
            right_success = self.right_arm.execute(blocking=True)

            success = left_success and right_success
            if success:
                self.get_logger().info('已移動到初始位姿')
            else:
                self.get_logger().error('移動到初始位姿失敗')
            return success
        else:
            self.get_logger().error('初始位姿規劃失敗')
            return False

    def get_current_left_pose(self):
        """獲取當前左手臂末端位姿"""
        robot_state = self.left_arm.get_start_state()
        # TODO: 實現獲取當前位姿的邏輯
        return None

    def get_current_right_pose(self):
        """獲取當前右手臂末端位姿"""
        robot_state = self.right_arm.get_start_state()
        # TODO: 實現獲取當前位姿的邏輯
        return None

    def _create_pose_stamped(self, pose, frame_id):
        """創建 PoseStamped 消息"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped


def demo_separate_control(controller):
    """示範分別控制左右手臂"""
    controller.get_logger().info('===== 示範：分別控制左右手臂 =====')

    # 創建左手臂目標位姿
    left_pose = Pose()
    left_pose.position.x = 0.3
    left_pose.position.y = 0.3
    left_pose.position.z = 1.2
    left_pose.orientation.w = 1.0

    # 創建右手臂目標位姿
    right_pose = Pose()
    right_pose.position.x = 0.3
    right_pose.position.y = -0.3
    right_pose.position.z = 1.2
    right_pose.orientation.w = 1.0

    # 先移動左手臂
    controller.get_logger().info('移動左手臂...')
    controller.move_left_arm_to_pose(left_pose)

    # 再移動右手臂
    controller.get_logger().info('移動右手臂...')
    controller.move_right_arm_to_pose(right_pose)


def demo_simultaneous_control(controller):
    """示範同時控制左右手臂"""
    controller.get_logger().info('===== 示範：同時控制左右手臂 =====')

    # 創建左手臂目標位姿
    left_pose = Pose()
    left_pose.position.x = 0.4
    left_pose.position.y = 0.25
    left_pose.position.z = 1.1
    left_pose.orientation.w = 1.0

    # 創建右手臂目標位姿
    right_pose = Pose()
    right_pose.position.x = 0.4
    right_pose.position.y = -0.25
    right_pose.position.z = 1.1
    right_pose.orientation.w = 1.0

    # 同時移動兩個手臂
    controller.move_both_arms_to_pose(left_pose, right_pose)


def demo_joint_control(controller):
    """示範關節角度控制"""
    controller.get_logger().info('===== 示範：關節角度控制 =====')

    # 左手臂關節角度
    left_joints = [0.0, 0.5, 1.0, 0.0, -0.3, 0.0]

    # 右手臂關節角度
    right_joints = [0.0, -0.5, -1.0, 0.0, 0.3, 0.0]

    # 移動左手臂
    controller.get_logger().info('通過關節角度移動左手臂...')
    controller.move_left_arm_to_joints(left_joints)

    # 移動右手臂
    controller.get_logger().info('通過關節角度移動右手臂...')
    controller.move_right_arm_to_joints(right_joints)


def main(args=None):
    """主函數"""
    rclpy.init(args=args)

    try:
        # 創建控制器
        controller = DualArmController()

        # 先移動到初始位姿
        controller.get_logger().info('移動到初始位姿...')
        controller.move_to_initial_pose()

        # 運行示範程序
        if len(sys.argv) > 1:
            mode = sys.argv[1]

            if mode == 'separate':
                demo_separate_control(controller)
            elif mode == 'simultaneous':
                demo_simultaneous_control(controller)
            elif mode == 'joint':
                demo_joint_control(controller)
            else:
                controller.get_logger().info('使用方式: simple_ee_control.py [separate|simultaneous|joint]')
        else:
            # 默認運行所有示範
            controller.get_logger().info('運行所有示範...')
            demo_separate_control(controller)
            demo_simultaneous_control(controller)
            demo_joint_control(controller)

        controller.get_logger().info('示範完成')

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'錯誤: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
