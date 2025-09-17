import numpy as np
from src.robot import Robot

class BeeSwarmAgent(Robot):
    """
    An agent in a swarm with two roles: Scout and Forager.
    Scouts explore, and when they find the target, they recruit Foragers.
    """
    def __init__(self, agent_id, start_pos, speed=1.0, deposit_rate=10.0, is_scout=False):
        super().__init__(agent_id, start_pos, "BeeSwarmPheromones", speed)
        self.deposit_rate = deposit_rate
        self.state = "Scouting" if is_scout else "Waiting" # Scouting, Foraging, ReturningToRecruit, Waiting
        self.search_persistence = 0.7

    def step(self, env, swarm_info=None):
        """Perform actions based on the agent's role (Scout or Forager)."""
        if self.status == "Found Target":
            if self.state in ["Scouting", "Foraging"]:
                self.state = "ReturningToRecruit"
            env.deposit_pheromone(self.pos, self.deposit_rate * 10) # Deposit large amount at target

        if self.state == "Scouting":
            self.explore(env)
            self.update_state(env.target_pos)
        elif self.state == "Foraging":
            self.follow_pheromones(env)
            self.update_state(env.target_pos)
        elif self.state == "ReturningToRecruit":
            self.return_to_nest(env)
        elif self.state == "Waiting":
            # Check for recruitment signal (strong pheromones near home)
            local_pheromones = env.sense_pheromones(env.home_pos, radius=5).sum()
            if local_pheromones > 100: # Recruitment threshold
                self.state = "Foraging"

    def explore(self, env):
        current_dir = np.array([np.cos(self.orientation), np.sin(self.orientation)])
        random_dir = np.random.randn(2)
        direction_vector = self.search_persistence * current_dir + (1 - self.search_persistence) * random_dir
        self.move(direction_vector, env)

    def follow_pheromones(self, env):
        pheromones = env.sense_pheromones(self.pos, radius=3)
        if pheromones.sum() > 1.0:
            grad_y, grad_x = np.gradient(pheromones)
            center_idx = pheromones.shape[0] // 2
            gradient_vector = np.array([grad_x[center_idx, center_idx], grad_y[center_idx, center_idx]])
            direction_vector = 0.9 * gradient_vector + 0.1 * np.random.randn(2)
        else: # If trail is lost, explore
            self.explore(env)
            return
        self.move(direction_vector, env)

    def return_to_nest(self, env):
        direction_vector = env.home_pos - self.pos
        self.move(direction_vector, env)
        env.deposit_pheromone(self.pos, self.deposit_rate * 2) # Leave strong trail back
        if np.linalg.norm(self.pos - env.home_pos) < 5:
            self.state = "Foraging" # Become a forager after recruiting
            self.status = "Searching" # Reset status to go find target again