"""Launch the Trigger A3 Isaac Sim command forwarder."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the A3 node with its YAML parameters."""
    params = FindPackageShare("hex_ros_isaacsim_chassis").find(
        "hex_ros_isaacsim_chassis") + "/config/ros2/trigger_a3_params.yaml"
    return LaunchDescription([Node(
        package="hex_ros_isaacsim_chassis",
        executable="isaacsim_trigger_a3",
        name="hex_ros_isaacsim_trigger_a3",
        output="screen",
        parameters=[params])])
