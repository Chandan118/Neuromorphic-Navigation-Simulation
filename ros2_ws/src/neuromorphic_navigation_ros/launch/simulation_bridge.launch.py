from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="neuromorphic_navigation_ros",
                executable="simulation_bridge",
                name="neuromorphic_navigation_bridge",
                output="screen",
                parameters=[
                    {
                        "algorithm": "PotentialField",
                        "config_path": "config/simulation_params.json",
                        "publish_interval": 0.1,
                    }
                ],
            )
        ]
    )
