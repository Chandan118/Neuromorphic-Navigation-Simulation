import numpy as np
from src.robot import Robot


class NeuromorphicSpikingNavigator(Robot):
    """
    Spiking neural navigation circuit that blends head-direction, place, and
    boundary-sensitive cells. Potentials integrate sensory evidence until a
    winning population spikes and drives the robot orientation.
    """

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        num_neurons=24,
        membrane_decay=0.85,
        sensory_weight=1.4,
        boundary_weight=0.9,
        inhibition_strength=0.6,
        refractory_period=6,
        noise_scale=0.03,
    ):
        super().__init__(agent_id, start_pos, "NeuromorphicSpiking", speed)
        self.num_neurons = num_neurons
        self.neuron_angles = np.linspace(0, 2 * np.pi, num_neurons, endpoint=False)
        self.membrane = np.zeros(num_neurons)
        self.spike_trace = np.zeros(num_neurons)
        self.refractory = np.zeros(num_neurons, dtype=int)
        self.membrane_decay = membrane_decay
        self.sensory_weight = sensory_weight
        self.boundary_weight = boundary_weight
        self.inhibition_strength = inhibition_strength
        self.refractory_period = refractory_period
        self.noise_scale = noise_scale
        self.state = "Exploring"

    def step(self, env, swarm_info=None):
        if self.status != "Searching" or env.target_pos is None:
            return

        # Compute sensory drive from target and obstacles
        target_vec = env.target_pos - self.pos
        target_dist = np.linalg.norm(target_vec)
        if target_dist < 1e-6:
            self.status = "Found Target"
            return
        target_dir = target_vec / target_dist

        boundary_drive = self._boundary_drive(env)

        preferred_dirs = np.stack(
            (np.cos(self.neuron_angles), np.sin(self.neuron_angles)), axis=1
        )

        sensory_current = self.sensory_weight * preferred_dirs @ target_dir
        boundary_current = self.boundary_weight * (preferred_dirs @ boundary_drive)

        # Update membrane potentials with decay, excitation, and inhibition
        self.membrane *= self.membrane_decay
        self.membrane += sensory_current + boundary_current
        self.membrane -= self.inhibition_strength * self.spike_trace

        # Refractory handling and spikes
        self.spike_trace *= 0.8
        active_neurons = self.membrane > 1.0
        for idx, is_active in enumerate(active_neurons):
            if not is_active:
                continue
            if self.refractory[idx] > 0:
                continue
            self.spike_trace[idx] = 1.0
            self.membrane[idx] = 0.2
            self.refractory[idx] = self.refractory_period

        self.refractory = np.maximum(self.refractory - 1, 0)

        # Convert population code into heading
        spike_weights = self.membrane + self.spike_trace
        softmax = np.exp(spike_weights - np.max(spike_weights))
        softmax /= np.sum(softmax) + 1e-8
        heading_vector = softmax @ preferred_dirs
        heading_norm = np.linalg.norm(heading_vector)
        if heading_norm < 1e-6:
            heading_vector = target_dir
        else:
            heading_vector /= heading_norm

        noise = np.random.randn(2) * self.noise_scale
        direction = heading_vector + noise
        self.move(direction, env)
        self.update_state(env.target_pos)

        self.energy_used += 0.07 + 0.02 * np.sum(active_neurons)
        self.state = "Reorienting" if active_neurons.any() else "Exploring"
        self.cumulative_drift = np.linalg.norm(self.pos - env.home_pos)

    def _boundary_drive(self, env):
        """
        Sample local obstacle layout to bias the agent away from collisions.
        """
        sample_angles = self.orientation + np.linspace(-np.pi / 2, np.pi / 2, 9)
        avoidance = np.zeros(2)
        for ang in sample_angles:
            direction = np.array([np.cos(ang), np.sin(ang)])
            for d in np.linspace(1.0, 6.0, 5):
                pt = self.pos + direction * d
                if not env.is_valid_position(pt):
                    avoidance -= direction / (d + 1e-6)
                    break
        norm = np.linalg.norm(avoidance)
        if norm < 1e-6:
            return np.zeros(2)
        return avoidance / norm
