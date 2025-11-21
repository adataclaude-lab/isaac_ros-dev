from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 可從外面傳入 XRDF / URDF 路徑
    robot_xrdf = LaunchConfiguration('robot_xrdf')
    robot_urdf = LaunchConfiguration('robot_urdf')

    declare_robot_xrdf = DeclareLaunchArgument(
        'robot_xrdf',
        description='Path to robot XRDF file'
    )

    declare_robot_urdf = DeclareLaunchArgument(
        'robot_urdf',
        description='Path to robot URDF file'
    )

    # 使用 isaac_ros_cumotion 提供的 cumotion_planner_node
    cumotion_params = PathJoinSubstitution([
        get_package_share_directory('arm_left_moveit_config'),
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
