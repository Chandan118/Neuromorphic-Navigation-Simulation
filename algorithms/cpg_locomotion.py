import numpy as np
from src.robot import Robot


class CPGNavigator(Robot):
    """
    Central pattern generator (CPG) based navigator that produces rhythmic gait
    patterns to drive motion and steering. Oscillators adapt their phase based on
    target direction and terrain feedback for bio-inspired locomotion.
    """

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        base_frequency=0.35,
        coupling_strength=0.2,
        sensory_gain=0.6,
        damping=0.05,
        stride_gain=1.1,
        obstacle_frequency_boost=0.25,
    ):
        super().__init__(agent_id, start_pos, "CPGNavigator", speed)
        self.phase_left = 0.0
        self.phase_right = np.pi
        self.base_frequency = base_frequency
        self.coupling_strength = coupling_strength
        self.sensory_gain = sensory_gain
        self.damping = damping
        self.stride_gain = stride_gain
        self.obstacle_frequency_boost = obstacle_frequency_boost
        self.state = "RhythmicMotion"

    def step(self, env, swarm_info=None):
        if self.status != "Searching" or env.target_pos is None:
            return

        target_vec = env.target_pos - self.pos
        distance = np.linalg.norm(target_vec)
        if distance > 0:
            target_dir = target_vec / distance
        else:
            target_dir = np.array([np.cos(self.orientation), np.sin(self.orientation)])

        obstacle_signal = self._terrain_feedback(env)
        adaptive_freq = self.base_frequency + self.obstacle_frequency_boost * obstacle_signal

        left_coupling = self.coupling_strength * np.sin(self.phase_right - self.phase_left)
        right_coupling = self.coupling_strength * np.sin(self.phase_left - self.phase_right)

        self.phase_left += adaptive_freq + left_coupling + self.sensory_gain * obstacle_signal
        self.phase_right += adaptive_freq + right_coupling - self.sensory_gain * obstacle_signal

        left_output = np.sin(self.phase_left)
        right_output = np.sin(self.phase_right)

        turn_signal = self.stride_gain * (right_output - left_output)
        self.orientation += turn_signal * 0.1 - self.damping * (self.orientation - np.arctan2(target_dir[1], target_dir[0]))

        direction = np.array([np.cos(self.orientation), np.sin(self.orientation)])
        step_vector = direction * (1 + 0.2 * (left_output + right_output))

        self.move(step_vector, env)
        self.update_state(env.target_pos)

        self.energy_used += 0.12 + 0.03 * abs(turn_signal)
        self.state = "ObstacleResponse" if obstacle_signal > 0 else "RhythmicMotion"

    def _terrain_feedback(self, env):
        lookahead = np.array([np.cos(self.orientation), np.sin(self.orientation)]) * 4.0
        sample_point = self.pos + lookahead
        if not env.is_valid_position(sample_point):
            return 1.0
        lateral = np.array([-lookahead[1], lookahead[0]])
        sample_left = self.pos + lateral
        sample_right = self.pos - lateral
        signal = 0.0
        if not env.is_valid_position(sample_left):
            signal += 0.5
        if not env.is_valid_position(sample_right):
            signal -= 0.5
        return signal
