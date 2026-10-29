# Lab 1, Part 3: starts rrbot (RViz + RQT + joint states), writes the rrbot
# URDF to a file with xacro, and then starts test_fk reading that file.

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

URDF_FILE = "/tmp/rrbot.urdf"


def generate_launch_description():
    xacro_file = PathJoinSubstitution([FindPackageShare("rrbot"), "urdf", "rrbot.urdf.xacro"])

    rrbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rrbot"), "launch", "rrbot.launch.py"])
        )
    )

    # xacro -> plain URDF file
    make_urdf = ExecuteProcess(
        cmd=["xacro", xacro_file, "-o", URDF_FILE],
        output="screen",
    )

    test_fk = Node(
        package="lab01",
        executable="test_fk",
        output="screen",
        parameters=[{"urdf_path": URDF_FILE}],
    )

    start_fk_after_urdf = RegisterEventHandler(
        OnProcessExit(target_action=make_urdf, on_exit=[test_fk])
    )

    return LaunchDescription([rrbot, make_urdf, start_fk_after_urdf])
