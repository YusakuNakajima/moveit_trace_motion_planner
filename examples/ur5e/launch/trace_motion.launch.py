from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParametersFromFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    trace_params = PathJoinSubstitution(
        [
            FindPackageShare("moveit_trace_motion_planner"),
            "examples",
            "ur5e",
            "config",
            "trace_motion_planning.params.yaml",
        ]
    )
    ur_moveit_launch = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR robot type passed to ur_moveit_config.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Whether ur_moveit_config should launch RViz.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Tell ur_moveit_config to use fake-hardware controller defaults.",
            ),
            GroupAction(
                [
                    SetParametersFromFile(trace_params),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([ur_moveit_launch]),
                        launch_arguments={
                            "ur_type": ur_type,
                            "launch_rviz": launch_rviz,
                            "use_fake_hardware": use_fake_hardware,
                        }.items(),
                    ),
                ]
            ),
        ]
    )
