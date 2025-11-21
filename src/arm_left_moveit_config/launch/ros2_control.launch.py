from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "arm_left_moveit_config"
    pkg_share = get_package_share_directory(pkg_name)

    # 1) 用 Command + xacro 產生 robot_description（標準寫法）
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution(
            [pkg_share, "config", "RobotArm_Left.urdf.xacro"]
        ),
    ])
    robot_description = {"robot_description": robot_description_content}

    # 2) ros2_control 的 yaml
    ros2_controllers_yaml = PathJoinSubstitution(
        [pkg_share, "config", "ros2_controllers.yaml"]
    )

    # 3) 啟動 ros2_control_node（controller_manager）
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_yaml,
        ],
        output="screen",
    )

    return LaunchDescription([ros2_control_node])
