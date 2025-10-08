import numpy as np
from src.robot import Robot


class DeepRLNavigator(Robot):
    """
    Lightweight deep Q-learning style navigator that learns online using a small
    neural network approximator. Mimics neuromorphic-inspired trial-and-error
    learning observed in rodents exploring new environments.
    """

    ACTIONS = np.array(
        [
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0],
            [0.0, -1.0],
            [0.707, 0.707],
            [-0.707, 0.707],
            [-0.707, -0.707],
            [0.707, -0.707],
        ]
    )

    def __init__(
        self,
        agent_id,
        start_pos,
        speed=1.0,
        hidden_size=32,
        learning_rate=0.005,
        gamma=0.92,
        epsilon=0.15,
        epsilon_decay=0.999,
    ):
        super().__init__(agent_id, start_pos, "DeepRL", speed)
        self.hidden_size = hidden_size
        self.learning_rate = learning_rate
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.state_dim = 10

        rng = np.random.default_rng(seed=42)
        self.W1 = rng.normal(scale=0.2, size=(self.state_dim, hidden_size))
        self.b1 = np.zeros(hidden_size)
        self.W2 = rng.normal(scale=0.2, size=(hidden_size, len(self.ACTIONS)))
        self.b2 = np.zeros(len(self.ACTIONS))

        self.prev_state = None
        self.prev_action = None
        self.prev_distance = None

    def step(self, env, swarm_info=None):
        if env.target_pos is None or self.status != "Searching":
            return

        state = self._collect_state(env)
        q_values, hidden = self._forward(state)

        if np.random.rand() < self.epsilon:
            action_idx = np.random.randint(len(self.ACTIONS))
        else:
            action_idx = int(np.argmax(q_values))

        direction = self.ACTIONS[action_idx]
        self.move(direction, env)
        self.update_state(env.target_pos)

        new_state = self._collect_state(env)
        new_q, _ = self._forward(new_state)

        reward = self._compute_reward(env)
        target = reward + self.gamma * np.max(new_q)

        if self.prev_state is not None and self.prev_action is not None:
            self._backpropagate(self.prev_state, self.prev_hidden, self.prev_q, self.prev_action, target)

        self.prev_state = state
        self.prev_hidden = hidden
        self.prev_q = q_values
        self.prev_action = action_idx
        self.prev_distance = np.linalg.norm(env.target_pos - self.pos)

        self.epsilon *= self.epsilon_decay
        self.energy_used += 0.1 + 0.05 * np.linalg.norm(direction)

    def _collect_state(self, env):
        target_vec = env.target_pos - self.pos
        distance = np.linalg.norm(target_vec)
        if distance > 0:
            target_dir = target_vec / distance
        else:
            target_dir = np.zeros(2)

        readings = []
        for offset in np.linspace(0, 2 * np.pi, 6, endpoint=False):
            direction = np.array([np.cos(self.orientation + offset), np.sin(self.orientation + offset)])
            reading = 0.0
            for d in np.linspace(1.0, 12.0, 6):
                point = self.pos + direction * d
                if not env.is_valid_position(point):
                    reading = 1.0 / (d + 1e-6)
                    break
            readings.append(reading)

        state = np.array(
            [
                target_dir[0],
                target_dir[1],
                np.clip(distance / (np.linalg.norm(env.size) + 1e-6), 0, 1),
                np.cos(self.orientation),
                np.sin(self.orientation),
                env.pheromone_grid[int(self.pos[0]) % env.size[0], int(self.pos[1]) % env.size[1]],
                *readings[:4],
            ]
        )
        return state

    def _forward(self, state):
        hidden = np.tanh(state @ self.W1 + self.b1)
        q_values = hidden @ self.W2 + self.b2
        return q_values, hidden

    def _backpropagate(self, state, hidden, q_values, action_idx, target):
        td_error = q_values[action_idx] - target
        grad_output = np.zeros_like(q_values)
        grad_output[action_idx] = td_error

        grad_W2 = np.outer(hidden, grad_output)
        grad_b2 = grad_output
        grad_hidden = self.W2 @ grad_output
        grad_hidden *= (1 - hidden**2)
        grad_W1 = np.outer(state, grad_hidden)
        grad_b1 = grad_hidden

        self.W2 -= self.learning_rate * grad_W2
        self.b2 -= self.learning_rate * grad_b2
        self.W1 -= self.learning_rate * grad_W1
        self.b1 -= self.learning_rate * grad_b1

    def _compute_reward(self, env):
        if env.target_pos is None:
            return 0.0
        distance = np.linalg.norm(env.target_pos - self.pos)
        prev = self.prev_distance if self.prev_distance is not None else distance + 1.0
        reward = (prev - distance) * 2.5
        reward -= 0.01 * self.energy_used
        if self.status == "Found Target":
            reward += 25.0
        return reward
