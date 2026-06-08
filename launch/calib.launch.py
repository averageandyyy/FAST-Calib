from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    parameter_sources = [params_file]

    overrides = {}
    for name in ("bag_path", "image_path", "output_path", "lidar_topic"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    if overrides:
        parameter_sources.append(overrides)

    rviz_config = PathJoinSubstitution([
        FindPackageShare("fast_calib"),
        "rviz_cfg",
        "fast_livo2.rviz",
    ])

    return [
        Node(
            package="fast_calib",
            executable="fast_calib",
            name="fast_calib",
            output="screen",
            parameters=parameter_sources,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare("fast_calib"),
        "config",
        "qr_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("bag_path", default_value=""),
        DeclareLaunchArgument("image_path", default_value=""),
        DeclareLaunchArgument("output_path", default_value=""),
        DeclareLaunchArgument("lidar_topic", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
