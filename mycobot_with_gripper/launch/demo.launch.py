from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("mycobot_320", package_name="mycobot_with_gripper")
                    .robot_description_kinematics()
                    .robot_description_semantic()
                    .joint_limits()
                    .to_moveit_configs()
                    )
    return generate_demo_launch(moveit_config)
