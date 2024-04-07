from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = {"use_sim_time": use_sim_time}

    node_motor_controller = Node(
        package="sixlg_motor_controller",
        executable="motor_controller",
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
            node_motor_controller,
        ]
    )
