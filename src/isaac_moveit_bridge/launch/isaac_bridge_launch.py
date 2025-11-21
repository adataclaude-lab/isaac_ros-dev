from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Isaac Sim) clock if true",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)

    # # Node to plan to a hardcoded pose and publish the trajectory
    # ld.add_action(
    #     Node(
    #         package="isaac_moveit_bridge",
    #         executable="plan_and_publish",
    #         output="screen",
    #         parameters=[
    #             {'use_sim_time': use_sim_time,}
    #         ],
    #     )
    # )

    # Node to subscribe to planned trajectories and execute them
    ld.add_action(
        Node(
            package="isaac_moveit_bridge",
            executable="trajectory_executor",
            output="screen",
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'path_tolerance_rad': 100.0,
                    'goal_tolerance_rad': 100.0,
                    'position_tolerance_rad': 0.05,
                }
            ],
        )
    )

    return ld
