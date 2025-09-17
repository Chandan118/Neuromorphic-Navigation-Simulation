import numpy as np

class Environment:
    """
    Manages the 2D simulation world, including obstacles, multiple targets, and pheromones.
    """
    def __init__(self, size, num_obstacles, targets_list, home_pos, pheromone_decay_rate):
        self.size = size
        self.grid = np.zeros(size)
        self.targets_list = [np.array(t) for t in targets_list]
        self.home_pos = np.array(home_pos)
        self.pheromone_decay_rate = pheromone_decay_rate

        # The primary target for swarm/baseline is the first in the list
        self.target_pos = self.targets_list[0] if self.targets_list else None

        # Place obstacles (value = -1)
        self.obstacles = []
        all_special_points = [tuple(t) for t in self.targets_list] + [tuple(home_pos)]
        for _ in range(num_obstacles):
            pos = (np.random.randint(0, self.size[0]), np.random.randint(0, self.size[1]))
            # Ensure obstacles are not on targets or home
            if pos not in all_special_points:
                self.grid[pos[0], pos[1]] = -1
                self.obstacles.append(pos)
        
        self.pheromone_grid = np.zeros(size)

    def is_valid_position(self, pos):
        """Check if a position is within bounds and not an obstacle."""
        x, y = int(pos[0]), int(pos[1])
        if 0 <= x < self.size[0] and 0 <= y < self.size[1]:
            return self.grid[x, y] != -1
        return False

    def deposit_pheromone(self, pos, amount):
        """Deposit pheromone at a given position."""
        x, y = int(pos[0]), int(pos[1])
        if self.is_valid_position(pos):
            self.pheromone_grid[x, y] += amount

    def sense_pheromones(self, pos, radius=2):
        """Sense pheromone concentrations in the vicinity of a position."""
        x, y = int(pos[0]), int(pos[1])
        x_min, x_max = max(0, x - radius), min(self.size[0], x + radius + 1)
        y_min, y_max = max(0, y - radius), min(self.size[1], y + radius + 1)
        return self.pheromone_grid[x_min:x_max, y_min:y_max]

    def update(self):
        """Update the environment state, e.g., pheromone evaporation."""
        self.pheromone_grid *= self.pheromone_decay_rate
        self.pheromone_grid[self.pheromone_grid < 0.1] = 0