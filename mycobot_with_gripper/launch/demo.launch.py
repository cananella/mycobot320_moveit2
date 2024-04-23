from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription


def generate_launch_description():
    res=[]
    moveit_config = (MoveItConfigsBuilder("mycobot_320", package_name="mycobot_with_gripper")
                    .robot_description_kinematics()
                    .robot_description()
                    .joint_limits()
                    .to_moveit_configs()
                    )
    res.append(generate_demo_launch(moveit_config))
    return LaunchDescription(res)