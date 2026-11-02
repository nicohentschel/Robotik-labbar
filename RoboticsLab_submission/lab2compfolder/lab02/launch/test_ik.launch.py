from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdfPath = "/tmp/rrbot.urdf"
    xacroFile = PathJoinSubstitution([FindPackageShare("rrbot"), "urdf", "rrbot.urdf.xacro"])
    rvizConfig = PathJoinSubstitution([FindPackageShare("rrbot"), "config", "rrbot.rviz"])

    #robot state publisher, listens to ik_joint_states
    robotStatePub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", xacroFile])}],
        remappings=[("joint_states", "ik_joint_states")],
    )

    #rviz
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", rvizConfig])

    #xacro -> urdf file
    makeUrdf = ExecuteProcess(cmd=["xacro", xacroFile, "-o", urdfPath])

    #ik node, starts after urdf file exists
    testIk = Node(
        package="lab02",
        executable="test_ik",
        parameters=[{"urdfPath": urdfPath}],
        output="screen",
    )
    startIk = RegisterEventHandler(OnProcessExit(target_action=makeUrdf, on_exit=[testIk]))

    return LaunchDescription([robotStatePub, rviz, makeUrdf, startIk])