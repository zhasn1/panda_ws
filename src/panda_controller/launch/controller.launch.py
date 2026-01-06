from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition

def generate_launch_description():
    pkg_share = get_package_share_directory("panda_controller")
    description_pkg_path = get_package_share_directory("panda_description")

    is_sim = LaunchConfiguration("is_sim")
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    description_pkg_path,
                    "urdf",
                    "panda.urdf.xacro",
                ),
                " is_sim:=True",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": is_sim,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(is_sim),
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": is_sim,
            },
            os.path.join(
                pkg_share,
                "config",
                "panda_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
