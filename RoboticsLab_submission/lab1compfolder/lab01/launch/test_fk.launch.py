from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdfPath = "/tmp/rrbot.urdf"
    xacroFile = PathJoinSubstitution([FindPackageShare("rrbot"), "urdf", "rrbot.urdf.xacro"])
    rrbotLaunch = PathJoinSubstitution([FindPackageShare("rrbot"), "launch", "rrbot.launch.py"])

    #rrbot: rviz + rqt + joint states
    rrbot = IncludeLaunchDescription(PythonLaunchDescriptionSource(rrbotLaunch))

    #xacro -> urdf file
    makeUrdf = ExecuteProcess(cmd=["xacro", xacroFile, "-o", urdfPath])

    #fk node, starts after urdf file exists
    testFk = Node(
        package="lab01",
        executable="test_fk",
        parameters=[{"urdfPath": urdfPath}],
        output="screen",
    )
    startFk = RegisterEventHandler(OnProcessExit(target_action=makeUrdf, on_exit=[testFk]))

    return LaunchDescription([rrbot, makeUrdf, startFk])
