#!/usr/bin/env python3
"""
雙手臂關節控制腳本（不使用 MoveIt）
直接發送關節軌跡指令
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys


class SimpleJointController(Node):
    """簡單的雙手臂關節控制器 - 不使用 MoveIt"""

    def __init__(self):
        super().__init__('simple_joint_controller')

        self.get_logger().info('初始化簡單關節控制器...')

        # 創建發布者 - 發送關節軌跡
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 等待發布者準備好
        self.get_logger().info('等待軌跡控制器連接...')

        self.get_logger().info('簡單關節控制器初始化完成')

    def move_to_position(self, left_joints, right_joints, duration_sec=2.0):
        """
        移動到指定關節位置

        Args:
            left_joints: list - 左臂 6 個關節角度 [j1, j2, j3, j4, j5, j6]
            right_joints: list - 右臂 6 個關節角度 [j1, j2, j3, j4, j5, j6]
            duration_sec: float - 移動持續時間（秒）
        """
        if len(left_joints) != 6 or len(right_joints) != 6:
            self.get_logger().error('需要 6 個關節角度（每隻手臂）')
            return False

        # 創建軌跡訊息
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 設置關節名稱
        msg.joint_names = [
            'j1_Joint_L', 'j2_Joint_L', 'j3_Joint_L',
            'j4_Joint_L', 'j5_Joint_L', 'j6_Joint_L',
            'j1_Joint_R', 'j2_Joint_R', 'j3_Joint_R',
            'j4_Joint_R', 'j5_Joint_R', 'j6_Joint_R'
        ]

        # 創建軌跡點
        point = JointTrajectoryPoint()
        point.positions = left_joints + right_joints
        point.time_from_start = Duration(sec=int(duration_sec),
                                        nanosec=int((duration_sec % 1) * 1e9))

        msg.points.append(point)

        # 發布軌跡
        self.get_logger().info(f'發送關節軌跡指令...')
        self.get_logger().info(f'左臂: {left_joints}')
        self.get_logger().info(f'右臂: {right_joints}')

        self.trajectory_pub.publish(msg)

        self.get_logger().info('軌跡已發送')
        return True

    def move_left_arm(self, joints, duration_sec=2.0):
        """只移動左臂"""
        right_zeros = [0.0] * 6
        return self.move_to_position(joints, right_zeros, duration_sec)

    def move_right_arm(self, joints, duration_sec=2.0):
        """只移動右臂"""
        left_zeros = [0.0] * 6
        return self.move_to_position(left_zeros, joints, duration_sec)


def main(args=None):
    """主函數"""
    rclpy.init(args=args)

    try:
        controller = SimpleJointController()

        # 示範：移動到指定關節位置
        if len(sys.argv) > 1:
            mode = sys.argv[1]

            if mode == 'home':
                # 回到零位
                controller.get_logger().info('移動到零位...')
                controller.move_to_position(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    duration_sec=3.0
                )

            elif mode == 'test':
                # 測試動作
                controller.get_logger().info('執行測試動作...')
                controller.move_to_position(
                    [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
                    [0.0, 0.5, -0.5, 0.0, -0.5, 0.0],
                    duration_sec=3.0
                )

            else:
                controller.get_logger().info('未知模式。使用: home, test')
        else:
            controller.get_logger().info('使用方式: simple_joint_control.py [home|test]')

        # 等待一段時間讓軌跡執行
        rclpy.spin_once(controller, timeout_sec=5.0)

        controller.get_logger().info('完成')

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
