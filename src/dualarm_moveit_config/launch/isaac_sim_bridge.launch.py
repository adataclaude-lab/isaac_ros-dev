from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bridge_config = os.path.join(
        get_package_share_directory('dualarm_description'),
        'config',
        'isaac_sim_bridge.yaml'
    )
    
    # Isaac ROS Bridge Node
    isaac_bridge = Node(
        package='isaac_ros_bridge',
        executable='isaac_ros_bridge_node',
        name='isaac_sim_bridge',
        output='screen',
        parameters=[bridge_config]
    )
    
    return LaunchDescription([isaac_bridge])
