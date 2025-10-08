import numpy as np
from src.robot import Robot


class PotentialFieldNavigator(Robot):
    """
    Gradient-based navigator that combines attractive forces towards the target
    with repulsive forces away from obstacles. Designed as a deterministic
    alternative to the stochastic baseline methods.
    """

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        attractive_gain=1.2,
        repulsive_gain=35.0,
        obstacle_influence_radius=10.0,
        damping=0.2,
        noise_scale=0.05,
    ):
        super().__init__(agent_id, start_pos, "PotentialField", speed)
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.obstacle_influence_radius = obstacle_influence_radius
        self.velocity = np.zeros(2, dtype=float)
        self.damping = damping
        self.noise_scale = noise_scale

    def step(self, env, swarm_info=None):
        if self.status != "Searching" or env.target_pos is None:
            return

        attractive_force = self._compute_attractive_force(env.target_pos)
        repulsive_force = self._compute_repulsive_force(env.obstacles)
        random_force = np.random.randn(2) * self.noise_scale

        net_force = attractive_force + repulsive_force + random_force
        self.velocity = (1 - self.damping) * self.velocity + net_force

        if np.linalg.norm(self.velocity) > 1e-6:
            direction_vector = self.velocity / np.linalg.norm(self.velocity)
        else:
            direction_vector = np.random.randn(2)

        self.move(direction_vector, env)
        self.update_state(env.target_pos)

    def _compute_attractive_force(self, target_pos):
        direction = np.array(target_pos, dtype=float) - self.pos
        distance = np.linalg.norm(direction)
        if distance < 1e-6:
            return np.zeros(2)
        direction /= distance
        return self.attractive_gain * direction

    def _compute_repulsive_force(self, obstacles):
        repulsive_force = np.zeros(2)
        if not obstacles:
            return repulsive_force

        for obs in obstacles:
            obs_vec = self.pos - np.array(obs, dtype=float)
            distance = np.linalg.norm(obs_vec)
            if distance < 1e-6 or distance > self.obstacle_influence_radius:
                continue
            direction = obs_vec / distance
            magnitude = self.repulsive_gain * ((1.0 / distance) - (1.0 / self.obstacle_influence_radius))
            repulsive_force += magnitude * direction

        return repulsive_force
