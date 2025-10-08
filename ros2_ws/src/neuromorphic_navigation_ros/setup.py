from setuptools import setup

package_name = "neuromorphic_navigation_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/simulation_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Neuromorphic Simulation",
    maintainer_email="support@example.com",
    description="ROS 2 bridge for the Neuromorphic Navigation Simulation project.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulation_bridge = neuromorphic_navigation_ros.bridge_node:main",
        ],
    },
)*** End Patch```
