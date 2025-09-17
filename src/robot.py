import numpy as np

class Robot:
    """
    Base class for a simulated agent.
    """
    def __init__(self, agent_id, start_pos, algorithm_name, speed=1.0):
        self.agent_id = agent_id
        self.pos = np.array(start_pos, dtype=float)
        self.algorithm = algorithm_name
        self.speed = speed
        self.orientation = np.random.uniform(0, 2 * np.pi)
        self.distance_to_target = float('inf')
        self.status = "Searching"
        self.state = "Default" # NEW: Internal state for more complex behavior
        self.energy_used = 0.0
        self.path = [tuple(start_pos)]
        # NEW attributes for logging
        self.cumulative_drift = 0.0
        self.landmarks_discovered = 0

    def move(self, direction_vector, env):
        """Move the robot in a given direction, respecting boundaries and obstacles."""
        if "Found" in self.status or "Returned" in self.status:
            return

        # Normalize direction and apply speed
        norm = np.linalg.norm(direction_vector)
        if norm > 0:
            direction_vector /= norm
        
        self.orientation = np.arctan2(direction_vector[1], direction_vector[0])
        
        new_pos = self.pos + direction_vector * self.speed
        
        # Simple collision avoidance: if new position is invalid, turn randomly
        if not env.is_valid_position(new_pos):
            self.orientation += np.random.uniform(-np.pi / 2, np.pi / 2)
            direction_vector = np.array([np.cos(self.orientation), np.sin(self.orientation)])
            new_pos = self.pos + direction_vector * self.speed

        if env.is_valid_position(new_pos):
            self.pos = new_pos
            self.path.append(tuple(self.pos))

        # Energy cost is proportional to distance moved
        self.energy_used += np.linalg.norm(direction_vector * self.speed)
        
    def update_state(self, target_pos):
        """Update agent's status based on proximity to the target."""
        self.distance_to_target = np.linalg.norm(self.pos - target_pos)
        if self.distance_to_target < 2.0 and self.status == "Searching":
            self.status = "Found Target"

    def step(self, env, swarm_info=None):
        """This method will be implemented by each specific navigation algorithm."""
        raise NotImplementedError("Each robot algorithm must implement the 'step' method.")