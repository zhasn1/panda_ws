from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("panda_moveit")
    description_pkg_share = get_package_share_directory("panda_description")

    is_sim = LaunchConfiguration("is_sim")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim", default_value="true", description="Use simulation time if true"
    )

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(
            file_path=os.path.join(description_pkg_share, "urdf", "panda.urdf.xacro"),
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Move Goup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
        output="screen",
    )

    rviz_config = os.path.join(
        pkg_share,
        "rviz",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": is_sim},
        ],
        output="screen",
    )

    return LaunchDescription([is_sim_arg, move_group_node, rviz_node])
