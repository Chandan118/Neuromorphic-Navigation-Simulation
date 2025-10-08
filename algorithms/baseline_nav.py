import numpy as np
from src.robot import Robot

class BaselineNavigator(Robot):
    """
    A robot that performs a simple random walk.
    Serves as a baseline for comparison.
    """
    def __init__(self, agent_id, start_pos, speed=1.0):
        super().__init__(agent_id, start_pos, "Baseline", speed)
        self.persistence = 0.8 # Tendency to keep moving in the same direction

    def step(self, env, swarm_info=None):
        """Perform one step of random walk navigation."""
        if self.status != "Searching":
            return
            
        # Create a new direction vector with some persistence from the old one
        current_dir = np.array([np.cos(self.orientation), np.sin(self.orientation)])
        random_dir = np.random.randn(2)
        if np.linalg.norm(random_dir) > 0:
            random_dir /= np.linalg.norm(random_dir)
        
        direction_vector = self.persistence * current_dir + (1 - self.persistence) * random_dir
        
        self.move(direction_vector, env)
        # For this simple agent, "distance_to_target" is against the first target
        if env.target_pos is not None:
            self.update_state(env.target_pos)