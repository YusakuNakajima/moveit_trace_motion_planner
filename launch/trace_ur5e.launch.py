from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    example_launch = PathJoinSubstitution(
        [
            FindPackageShare("moveit_trace_motion_planner"),
            "examples",
            "ur5e",
            "launch",
            "trace_motion.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([example_launch]),
                launch_arguments={
                    "ur_type": ur_type,
                    "launch_rviz": launch_rviz,
                    "use_fake_hardware": use_fake_hardware,
                }.items(),
            ),
        ]
    )
