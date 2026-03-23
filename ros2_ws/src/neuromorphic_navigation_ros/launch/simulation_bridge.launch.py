"""
simulation_bridge.launch.py

Author      : Chandan Sheikder
Email       : chandan@bit.edu.cn
Phone       : +8618222390506
Affiliation : Beijing Institute of Technology (BIT)
Date        : 2026-03-23

Description:
    Module for Simulation Bridge.Launch
"""

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
