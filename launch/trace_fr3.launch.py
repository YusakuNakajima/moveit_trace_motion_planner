from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    example_launch = PathJoinSubstitution(
        [
            FindPackageShare("moveit_trace_motion_planner"),
            "examples",
            "fr3",
            "launch",
            "trace_motion.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="dont-care"),
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([example_launch]),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "use_fake_hardware": use_fake_hardware,
                }.items(),
            ),
        ]
    )
