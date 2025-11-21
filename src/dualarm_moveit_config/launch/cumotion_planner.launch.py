from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    robot_xrdf = LaunchConfiguration('robot_xrdf')
    robot_urdf = LaunchConfiguration('robot_urdf')

    isaac_ws = os.environ.get("ISAAC_ROS_WS", "/workspaces/isaac_ros-dev")
    
    declare_robot_xrdf = DeclareLaunchArgument(
        'robot_xrdf',
        default_value=os.path.join(isaac_ws, 'src', 'dualarm_description', 'xrdf', 'dualarm.xrdf'),
        description='Path to dual-arm robot XRDF file'
    )

    declare_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        default_value=os.path.join(isaac_ws, 'src', 'dualarm_description', 'urdf', 'arm_dualarm.urdf'),
        description='Path to dual-arm robot URDF file'
    )

    cumotion_params = PathJoinSubstitution([
        get_package_share_directory('dualarm_moveit_config'),
        'config',
        'cumotion_parameters.yaml'
    ])

    cumotion_node = Node(
        package='isaac_ros_cumotion',
        executable='cumotion_planner_node',
        name='cumotion_planner_node',
        output='screen',
        parameters=[
            cumotion_params,
            {
                'robot': robot_xrdf,
                'urdf_path': robot_urdf,
            }
        ]
    )

    return LaunchDescription([
        declare_robot_xrdf,
        declare_robot_urdf,
        cumotion_node,
    ])
