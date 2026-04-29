from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    fake_hardware_launch = PathJoinSubstitution(
        [
            FindPackageShare("moveit_trace_motion_planner"),
            "examples",
            "ur5e",
            "launch",
            "fake_hardware.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([fake_hardware_launch]),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "launch_rviz": launch_rviz,
                    "initial_joint_controller": initial_joint_controller,
                }.items(),
            ),
        ]
    )
