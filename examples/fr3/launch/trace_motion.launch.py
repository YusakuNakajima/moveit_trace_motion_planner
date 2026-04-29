from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParametersFromFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    trace_params = PathJoinSubstitution(
        [
            FindPackageShare("moveit_trace_motion_planner"),
            "examples",
            "fr3",
            "config",
            "trace_motion_planning.params.yaml",
        ]
    )
    fr3_moveit_launch = PathJoinSubstitution(
        [FindPackageShare("franka_fr3_moveit_config"), "launch", "moveit.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="dont-care",
                description="FCI robot IP. Use dont-care with fake hardware.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use fake hardware for FR3 MoveIt demo.",
            ),
            GroupAction(
                [
                    SetParametersFromFile(trace_params),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([fr3_moveit_launch]),
                        launch_arguments={
                            "robot_ip": robot_ip,
                            "use_fake_hardware": use_fake_hardware,
                        }.items(),
                    ),
                ]
            ),
        ]
    )
