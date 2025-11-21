from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'dualarm_moveit_config'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rsp.launch.py'])
        )
    )

    # 2. Static Virtual Joint TFs
    static_virtual_joint_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'static_virtual_joint_tfs.launch.py'])
        )
    )

    # 3. cuMotion Planner Node
    isaac_ws = os.environ.get("ISAAC_ROS_WS", "/workspaces/isaac_ros-dev")
    cumotion_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'cumotion_planner.launch.py'])
        ),
        launch_arguments={
            'robot_xrdf': os.path.join(isaac_ws, 'src', 'dualarm_description', 'xrdf', 'dualarm.xrdf'),
            'robot_urdf': os.path.join(isaac_ws, 'src', 'dualarm_description', 'urdf', '2026_GTC_Robot_1120.urdf'),
        }.items()
    )

    # 4. MoveIt Move Group
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
        cumotion_planner,
        move_group,
        moveit_rviz,
    ])
