"""Launch the Maver X4 Isaac Sim command forwarder."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the X4 node with its YAML parameters."""
    params = FindPackageShare("hex_ros_isaacsim_chassis").find(
        "hex_ros_isaacsim_chassis") + "/config/ros2/maver_x4_params.yaml"
    return LaunchDescription([Node(
        package="hex_ros_isaacsim_chassis",
        executable="isaacsim_maver_x4",
        name="hex_ros_isaacsim_maver_x4",
        output="screen",
        parameters=[params])])
