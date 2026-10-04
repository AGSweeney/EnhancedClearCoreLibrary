from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    axis_mask = LaunchConfiguration("axis_mask")
    share = FindPackageShare("clearcore_hardware")
    xacro_file = PathJoinSubstitution([share, "urdf", "clearcore.urdf.xacro"])
    controllers = PathJoinSubstitution([share, "config", "controllers.yaml"])
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " host:=",
            host,
            " axis_mask:=",
            axis_mask,
        ])
    }
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="192.168.0.109"),
        DeclareLaunchArgument("axis_mask", default_value="3"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers, robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        ),
    ])
