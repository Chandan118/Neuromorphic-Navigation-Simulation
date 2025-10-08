import numpy as np
from src.robot import Robot


class SwarmConsensusAgent(Robot):
    """
    Distributed swarm navigator that combines cohesion, alignment, separation,
    and shared gradient sensing. Designed for soft leadership without a
    centralized controller.
    """

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        neighborhood_radius=20.0,
        cohesion_gain=0.04,
        alignment_gain=0.06,
        separation_gain=0.2,
        target_gain=0.9,
        noise_scale=0.08,
        velocity_damping=0.6,
    ):
        super().__init__(agent_id, start_pos, "SwarmConsensus", speed)
        self.neighborhood_radius = neighborhood_radius
        self.cohesion_gain = cohesion_gain
        self.alignment_gain = alignment_gain
        self.separation_gain = separation_gain
        self.target_gain = target_gain
        self.noise_scale = noise_scale
        self.velocity_damping = velocity_damping
        self.velocity = np.zeros(2)
        self.state = "Cohesion"

    def step(self, env, swarm_info=None):
        if env.target_pos is None or self.status != "Searching":
            return

        target_vec = env.target_pos - self.pos
        dist_to_target = np.linalg.norm(target_vec)
        if dist_to_target < 2.5:
            self.status = "Found Target"
            self.state = "Goal"
            return
        target_dir = target_vec / (dist_to_target + 1e-8)

        neighbors = swarm_info["agents"] if swarm_info else []

        cohesion = np.zeros(2)
        alignment = np.zeros(2)
        separation = np.zeros(2)
        neighbor_count = 0

        for neighbor in neighbors:
            offset = neighbor["pos"] - self.pos
            distance = np.linalg.norm(offset)
            if distance > self.neighborhood_radius or distance < 1e-6:
                continue
            neighbor_count += 1
            cohesion += offset
            alignment += neighbor["velocity"]
            separation -= offset / (distance**2 + 1e-6)

        if neighbor_count > 0:
            cohesion /= neighbor_count
            alignment /= neighbor_count

        noise = np.random.randn(2) * self.noise_scale
        drive = (
            self.target_gain * target_dir
            + self.cohesion_gain * cohesion
            + self.alignment_gain * alignment
            + self.separation_gain * separation
            + noise
        )

        if np.linalg.norm(drive) > 1e-6:
            drive /= np.linalg.norm(drive)

        self.velocity = (1 - self.velocity_damping) * self.velocity + self.velocity_damping * drive
        self.move(self.velocity, env)
        self.update_state(env.target_pos)

        self.state = "Coordinating" if neighbor_count else "Exploring"
        self.energy_used += np.linalg.norm(self.velocity) * 0.5
