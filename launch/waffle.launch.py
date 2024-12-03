
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name="name",
            default_value="waffle1",
            description="name of the ugv"
        ),

        DeclareLaunchArgument(
            name="model",
            default_value="waffle",
            description="model of the TurtleBot3"
        ),

        Node(
            package="ros2_differential_drive_line_following",
            executable="server",
            name=PythonExpression(["'", LaunchConfiguration('name'), "_line_following_action_server'"]),
            parameters=[
                {
                    'name': LaunchConfiguration('name'),
                    'model': LaunchConfiguration('model')
                }
            ],
            output="screen"
        ),
    ])
