from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    host = LaunchConfiguration("host").perform(context)
    axis_mask = int(LaunchConfiguration("axis_mask").perform(context))
    return [Node(
        package="clearcore_bridge",
        executable="bridge",
        name="clearcore_bridge",
        parameters=[{
            "host": host,
            "session_port": 9200,
            "stream_port": 9201,
            "axis_mask": axis_mask,
        }],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="192.168.0.109"),
        DeclareLaunchArgument("axis_mask", default_value="1"),
        OpaqueFunction(function=_launch),
    ])
