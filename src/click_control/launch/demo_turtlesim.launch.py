from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",
    )

    control_window = Node(
        package="click_control",
        executable="control_window",
        name="control_window",
        output="screen",
        remappings=[
            ("/cmd_vel", "/turtle1/cmd_vel"),
        ],
    )

    return LaunchDescription([turtlesim, control_window])
