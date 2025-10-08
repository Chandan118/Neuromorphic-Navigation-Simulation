import numpy as np
from src.robot import Robot


class ArtificialChemotaxisNavigator(Robot):
    """
    Navigation strategy that follows diffusing chemical gradients while laying
    down its own virtual reactants. Inspired by ant pheromone trails and
    slime-mold chemotaxis.
    """

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        gradient_gain=1.2,
        trail_deposit=4.0,
        noise_scale=0.08,
        desensitisation=0.92,
        gradient_radius=3,
        avoidance_gain=0.6,
    ):
        super().__init__(agent_id, start_pos, "ArtificialChemotaxis", speed)
        self.gradient_gain = gradient_gain
        self.trail_deposit = trail_deposit
        self.noise_scale = noise_scale
        self.desensitisation = desensitisation
        self.gradient_radius = gradient_radius
        self.avoidance_gain = avoidance_gain
        self.internal_bias = np.zeros(2)

    def step(self, env, swarm_info=None):
        if self.status != "Searching" or env.target_pos is None:
            return

        env.deposit_chemical(self.pos, self.trail_deposit)

        chemo_patch = env.sense_chemical(self.pos, radius=self.gradient_radius)
        if chemo_patch.size == 0:
            gradient = env.target_pos - self.pos
        else:
            grad_y, grad_x = np.gradient(chemo_patch)
            center = grad_x.shape[0] // 2
            gradient = np.array([grad_x[center, center], grad_y[center, center]])

        target_vec = env.target_pos - self.pos
        if np.linalg.norm(target_vec) > 0:
            target_vec /= np.linalg.norm(target_vec)

        obstacle_push = self._avoid_obstacles(env)
        noise = np.random.randn(2) * self.noise_scale

        drive = (
            self.gradient_gain * gradient
            + self.avoidance_gain * obstacle_push
            + 0.5 * target_vec
            + self.internal_bias
            + noise
        )
        norm = np.linalg.norm(drive)
        if norm > 1e-6:
            drive /= norm
        else:
            drive = target_vec if np.linalg.norm(target_vec) else np.random.randn(2)

        self.move(drive, env)
        self.update_state(env.target_pos)

        # adapt bias to recent motion (habituation)
        self.internal_bias = self.desensitisation * self.internal_bias + (1 - self.desensitisation) * drive
        self.energy_used += 0.08 + 0.02 * np.linalg.norm(drive)

    def _avoid_obstacles(self, env):
        offsets = np.array(
            [
                [1, 0],
                [-1, 0],
                [0, 1],
                [0, -1],
                [1, 1],
                [-1, 1],
                [1, -1],
                [-1, -1],
            ]
        )
        avoidance = np.zeros(2)
        for offset in offsets:
            sample = self.pos + offset * 2.0
            if not env.is_valid_position(sample):
                avoidance -= offset
        norm = np.linalg.norm(avoidance)
        if norm < 1e-6:
            return np.zeros(2)
        return avoidance / norm
