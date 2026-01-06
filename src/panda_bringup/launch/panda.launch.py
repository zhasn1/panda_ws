import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    description_pkg_share = get_package_share_directory("panda_description")
    controller_pkg_share = get_package_share_directory("panda_controller")
    moveit_pkg_share = get_package_share_directory("panda_moveit")

    # ======= Gazebo =======
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg_share,
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    # ======= Controller =======
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_pkg_share,
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ======= MoveIt2 =======
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_pkg_share,
                "launch",
                "moveit.launch.py",
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ======= Vision Node =======
    vision_node = Node(
        package="panda_vision",
        executable="color_detector",
        name="color_detector",
        output="screen",
    )

    # ======= PyMoveIt2 Picker Node =======
    # pick_and_place_node = Node(
    #     package="pymoveit2",
    #     executable="pick_and_place.py",
    #     name="pick_and_place_node",
    #     output="screen",
    #     parameters=[{"target_color": "B"}],
    #     prefix=["xterm -e "],
    # )

    return LaunchDescription(
        [
            gazebo,
            controller,
            moveit,
            vision_node,
            #pick_and_place_node,
        ]
    )
