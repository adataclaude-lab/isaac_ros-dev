from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Use dualarm_moveit_config instead of dualarm_description
    bridge_config = os.path.join(
        get_package_share_directory('dualarm_moveit_config'),
        'config',
        'isaac_sim_bridge.yaml'
    )

    # Isaac MoveIt Bridge Node (corrected package name)
    isaac_bridge = Node(
        package='isaac_moveit_bridge',
        executable='trajectory_executor',
        name='isaac_sim_bridge',
        output='screen',
        parameters=[bridge_config]
    )

    return LaunchDescription([isaac_bridge])
