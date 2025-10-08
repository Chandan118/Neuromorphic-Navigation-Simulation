import numpy as np
from src.robot import Robot


class SoftBodyHapticNavigator(Robot):
    """
    Morphology-inspired navigator that emulates a compliant soft robot gliding
    through clutter using tactile sensing. The agent steers towards the goal
    when unobstructed, but blends in surface-following behaviour whenever the
    virtual body detects contact forces around its perimeter.
    """

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        sensing_arc=np.pi,
        num_sensors=7,
        sensor_range=8.0,
        compliance=0.65,
        orientation_damping=0.35,
        contact_bias=0.5,
        exploration_noise=0.05,
    ):
        super().__init__(agent_id, start_pos, "SoftBodyHaptic", speed)
        self.sensing_arc = sensing_arc
        self.num_sensors = num_sensors
        self.sensor_range = sensor_range
        self.compliance = compliance
        self.orientation_damping = orientation_damping
        self.contact_bias = contact_bias
        self.exploration_noise = exploration_noise
        self.state = "Gliding"

    def step(self, env, swarm_info=None):
        if self.status != "Searching" or env.target_pos is None:
            return

        target_vector = env.target_pos - self.pos
        desired_angle = np.arctan2(target_vector[1], target_vector[0])

        contact_vector = self._sense_contacts(env)
        if contact_vector is not None:
            avoidance_angle = np.arctan2(contact_vector[1], contact_vector[0])
            # Blend goal-seeking with contact-driven boundary following
            desired_angle = (1 - self.compliance) * desired_angle + self.compliance * avoidance_angle
            self.state = "SurfaceFollowing"
        else:
            self.state = "Gliding"

        angle_error = (desired_angle - self.orientation + np.pi) % (2 * np.pi) - np.pi
        noise = np.random.normal(0, self.exploration_noise)
        self.orientation += self.orientation_damping * angle_error + noise

        direction = np.array([np.cos(self.orientation), np.sin(self.orientation)])
        self.move(direction, env)
        self.update_state(env.target_pos)

        heading_change_cost = abs(angle_error) * 0.1
        self.energy_used += 0.05 + heading_change_cost

    def _sense_contacts(self, env):
        """Cast virtual tactile rays and return an averaged repulsion vector."""
        sensor_angles = np.linspace(
            -self.sensing_arc / 2, self.sensing_arc / 2, self.num_sensors
        )

        contact_vectors = []
        for offset in sensor_angles:
            ray_angle = self.orientation + offset
            ray_dir = np.array([np.cos(ray_angle), np.sin(ray_angle)])

            samples = np.linspace(0.5, self.sensor_range, max(2, int(self.sensor_range)))
            for distance in samples:
                sample_point = self.pos + ray_dir * distance
                if not env.is_valid_position(sample_point):
                    # Push away from the obstacle; bias toward following the boundary
                    contact = self.pos - sample_point
                    if np.linalg.norm(contact) > 1e-6:
                        contact /= np.linalg.norm(contact)
                    tangential = np.array([-ray_dir[1], ray_dir[0]])
                    blended = (1 - self.contact_bias) * contact + self.contact_bias * tangential
                    contact_vectors.append(blended)
                    break

        if not contact_vectors:
            return None

        resultant = np.mean(contact_vectors, axis=0)
        norm = np.linalg.norm(resultant)
        if norm < 1e-6:
            return None
        return resultant / norm
