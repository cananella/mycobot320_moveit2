import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    res=[]
    moveit_config = (MoveItConfigsBuilder("mycobot_320", package_name="mycobot_with_gripper")
                    .robot_description_kinematics()
                    .robot_description()
                    .joint_limits()
                    .to_moveit_configs()
                    )
    res.append(generate_demo_launch(moveit_config))

    rviz_control_node = Node(
        package="mycobot_ros2_controll",
        executable="rviz_controll",
        name="rviz_controll",
        output="screen"
    )
    res.append(rviz_control_node)

    # cam_node = Node(
    #     package="mycobot_ros2_controll",
    #     executable="cam_pub",
    #     name="cam_pub",
    #     output="screen"
    # )
    # res.append(cam_node)

    return LaunchDescription(res)