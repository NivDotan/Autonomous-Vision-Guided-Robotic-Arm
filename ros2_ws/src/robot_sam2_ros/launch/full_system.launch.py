from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_daemon_arg = DeclareLaunchArgument(
        "use_daemon", default_value="false",
        description="Route motor commands through the C++ daemon instead of direct Python SDK")

    vision_node = Node(
        package="robot_sam2_ros",
        executable="vision_node",
        name="vision_node",
        output="screen",
    )
    kinematics_node = Node(
        package="robot_sam2_ros",
        executable="kinematics_node",
        name="kinematics_node",
        output="screen",
    )
    hardware_node = Node(
        package="robot_sam2_ros",
        executable="hardware_node",
        name="hardware_node",
        output="screen",
        parameters=[{"use_daemon": LaunchConfiguration("use_daemon")}],
    )
    sim_node = Node(
        package="robot_sam2_ros",
        executable="sim_node",
        name="sim_node",
        output="screen",
    )

    return LaunchDescription([
        use_daemon_arg,
        hardware_node,
        sim_node,
        kinematics_node,
        vision_node,
    ])
