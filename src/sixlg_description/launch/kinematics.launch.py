from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = {"use_sim_time": use_sim_time}

    node_kinematics = Node(
        package="sixlg_kinematics",
        executable="kinematics",
        output="screen",
        parameters=[params],
    )

    node_joint_state_publisher = Node(
        package="sixlg_kinematics",
        executable="joint_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            node_kinematics,
            node_joint_state_publisher,
        ]
    )
