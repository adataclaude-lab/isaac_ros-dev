from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("RobotArm_Left", package_name="arm_left_moveit_config")
        .planning_pipelines(
            default_planning_pipeline="isaac_ros_cumotion",
            pipelines=["isaac_ros_cumotion", "ompl"],
        )
        # 這一行一定要在
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config)
