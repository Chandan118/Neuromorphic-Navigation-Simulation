import copy
import json
import math
import os
import sys
import threading
from pathlib import Path
from typing import Tuple

PROJECT_ROOT_ENV = "NEUROMORPHIC_SIM_ROOT"

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import String
except ImportError as import_error:  # pragma: no cover - handled at runtime
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]
    PoseStamped = Path = String = None  # type: ignore[assignment]
    _ROS_IMPORT_ERROR = import_error
else:
    _ROS_IMPORT_ERROR = None


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert a planar yaw angle into a quaternion."""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class SimulationBridge(Node):
    """ROS 2 node that runs the Python simulation and publishes agent state."""

    def __init__(self) -> None:
        if _ROS_IMPORT_ERROR is not None:  # pragma: no cover - safety guard
            raise RuntimeError(
                "ROS 2 dependencies could not be imported. "
                "Ensure this node executes within a ROS 2 environment."
            ) from _ROS_IMPORT_ERROR

        super().__init__("neuromorphic_navigation_bridge")

        self.declare_parameter("algorithm", "PotentialField")
        self.declare_parameter("config_path", "config/simulation_params.json")
        self.declare_parameter("publish_interval", 0.1)
        self.declare_parameter("frame_id", "map")

        self.algorithm = self.get_parameter("algorithm").get_parameter_value().string_value
        config_value = self.get_parameter("config_path").get_parameter_value().string_value
        self.publish_interval = (
            self.get_parameter("publish_interval").get_parameter_value().double_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.config_path = Path(config_value)
        if not self.config_path.is_absolute():
            candidate_root = os.environ.get(PROJECT_ROOT_ENV, "")
            if candidate_root:
                self.config_path = Path(candidate_root) / self.config_path
            else:
                self.config_path = Path.cwd() / self.config_path

        if not self.config_path.exists():
            raise FileNotFoundError(
                f"Simulation config not found at {self.config_path}. "
                "Adjust the 'config_path' parameter or set NEUROMORPHIC_SIM_ROOT."
            )

        self.project_root = self.config_path.parent.parent
        env_module, simulation_module = self._import_core_modules()
        self.Environment = env_module.Environment  # type: ignore[attr-defined]
        self.Simulation = simulation_module.Simulation  # type: ignore[attr-defined]

        self.sim_params = self._load_config(self.config_path)

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.pose_pub = self.create_publisher(PoseStamped, "pose", qos_profile)
        self.path_pub = self.create_publisher(Path, "path", qos_profile)
        self.status_pub = self.create_publisher(String, "status", 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.step_lock = threading.Lock()
        self.last_published_step = -1

        self.get_logger().info(
            f"Bridge starting with algorithm '{self.algorithm}' using config at {self.config_path}"
        )

        self.simulation_thread = threading.Thread(target=self._run_simulation, daemon=True)
        self.simulation_thread.start()

    def _import_core_modules(self):
        """Dynamically import the Python simulation modules."""
        project_root = os.environ.get(PROJECT_ROOT_ENV)
        search_root = (
            Path(project_root).resolve() if project_root else self.project_root.resolve()
        )
        if str(search_root) not in sys.path:
            sys.path.append(str(search_root))
        from src import environment as environment_module  # type: ignore
        from src import simulation as simulation_module  # type: ignore

        return environment_module, simulation_module

    def _load_config(self, path: Path) -> dict:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _run_simulation(self) -> None:
        try:
            env_params = self.sim_params["environment"]
            algo_params = self.sim_params["algorithms"]

            env = self.Environment(
                size=tuple(env_params["size"]),
                num_obstacles=env_params["num_obstacles"],
                targets_list=env_params["targets_list"],
                home_pos=tuple(env_params["home_pos"]),
                pheromone_decay_rate=algo_params.get("bee_swarm", {}).get(
                    "pheromone_decay_rate", 0.99
                ),
            )

            sim = self.Simulation(
                env, self.algorithm, self.sim_params, show_progress=False
            )
            sim.register_step_callback(self._on_step)
            sim.run()
            self.get_logger().info("Simulation completed; logs written to results directory.")
        except Exception as exc:  # pragma: no cover - runtime safeguard
            self.get_logger().error(f"Simulation bridge failed: {exc}")

    def _on_step(self, step, agent, env) -> None:
        """Publish pose, path, and status information for each step."""
        if step == self.last_published_step:
            return
        self.last_published_step = step

        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = float(agent.pos[0])
        pose_msg.pose.position.y = float(agent.pos[1])
        pose_msg.pose.position.z = 0.0
        quat = yaw_to_quaternion(float(agent.orientation))
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        status_msg = String()
        status_msg.data = (
            f"{agent.algorithm} | step={step} | state={agent.state} | "
            f"status={agent.status} | energy={agent.energy_used:.2f}"
        )

        with self.step_lock:
            self.pose_pub.publish(pose_msg)
            self.status_pub.publish(status_msg)

            path_pose = PoseStamped()
            path_pose.header = pose_msg.header
            path_pose.pose = pose_msg.pose
            self.path_msg.header.stamp = stamp
            self.path_msg.poses.append(copy.deepcopy(path_pose))
            self.path_pub.publish(self.path_msg)


def main(args=None):
    if rclpy is None:  # pragma: no cover - handled by ROS launcher
        raise RuntimeError(
            "rclpy is unavailable. Ensure this entry point runs inside a ROS 2 environment."
        )

    rclpy.init(args=args)
    node = SimulationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - user-controlled exit
        node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
