from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'arm_left_moveit_config'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. robot_state_publisher + joint_state_broadcaster（rsp.launch.py）
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rsp.launch.py'])
        )
    )

    # 2. static virtual joint TFs（如果你有這個檔案）
    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'static_virtual_joint_tfs.launch.py'])
        )
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'ros2_control.launch.py'])
        )
    )

    # 3. 控制器（ros2_control controllers）
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'spawn_controllers.launch.py'])
        )
    )

    isaac_ws = os.environ.get("ISAAC_ROS_WS", "/workspaces/isaac_ros-dev")
    # 5. cuMotion planner node（自家包的 launch）
    cumotion_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'cumotion_planner.launch.py'])
        ),
        launch_arguments={
            # 這兩個路徑請改成你實際的絕對路徑
            'robot_xrdf': os.path.join(isaac_ws, 'src', 'arm_left_description', 'xrdf', 'arm_left.xrdf'),
            'robot_urdf': os.path.join(isaac_ws, 'src', 'arm_left_description', 'urdf', 'arm_left.urdf'),
        }.items()
    )

    # 4. MoveIt move_group（這裡面要已經有 cuMotion 的 planning_pipelines 設定）
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'move_group.launch.py'])
        )
    )

    # 5. MoveIt RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'moveit_rviz.launch.py'])
        )
    )

    return LaunchDescription([
        rsp,
        static_virtual_joint_tfs,
        ros2_control,
        spawn_controllers,
        cumotion_planner,
        move_group,
        moveit_rviz,
    ])
