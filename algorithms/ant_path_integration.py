import numpy as np
from src.robot import Robot

class AntPathIntegrator(Robot):
    """
    Implements desert ant path integration for a multi-target tour.
    """
    def __init__(self, agent_id, start_pos, speed=1.0, drift_scale=0.01, search_spiral_rate=0.1):
        super().__init__(agent_id, start_pos, "AntPathIntegration", speed)
        self.home_vector = np.zeros(2)
        self.perceived_pos = np.array(start_pos, dtype=float)
        self.drift_scale = drift_scale
        self.state = "Foraging"
        self.search_radius = 0
        self.search_angle = 0
        self.search_spiral_rate = search_spiral_rate
        self.target_idx = 0

    def step(self, env, swarm_info=None):
        if self.status == "Tour Complete":
            return

        direction_vector = np.zeros(2)
        
        # --- LOGIC RESTRUCTURED TO PREVENT INDEXERROR ---
        if self.state == "Foraging":
            # Only access the targets_list when foraging
            current_target = env.targets_list[self.target_idx]
            self.distance_to_target = np.linalg.norm(self.pos - current_target)
            direction_vector = current_target - self.pos
            
            # Check if target is reached
            if self.distance_to_target < 2.0:
                self.target_idx += 1
                # Check if the tour is over
                if self.target_idx >= len(env.targets_list):
                    self.state = "ReturningHome"
        
        elif self.state == "ReturningHome":
            direction_vector = -self.home_vector
            self.distance_to_target = np.linalg.norm(self.pos - env.home_pos)
            if np.linalg.norm(self.home_vector) < 2.0:
                if np.linalg.norm(self.pos - env.home_pos) > 5.0:
                    self.state = "Lost"
                else:
                    self.status = "Tour Complete"
                    
        elif self.state == "Lost":
            self.distance_to_target = np.linalg.norm(self.pos - env.home_pos)
            self.search_radius += self.search_spiral_rate * 0.1
            self.search_angle += self.search_spiral_rate
            direction_vector[0] = self.search_radius * np.cos(self.search_angle)
            direction_vector[1] = self.search_radius * np.sin(self.search_angle)
            if self.distance_to_target < 3.0:
                self.status = "Tour Complete"
        # --- END OF RESTRUCTURED LOGIC ---

        self.move_with_drift(direction_vector, env)
        
    def move_with_drift(self, direction_vector, env):
        norm = np.linalg.norm(direction_vector)
        if norm > 0:
            direction_vector /= norm

        actual_movement = direction_vector * self.speed
        super().move(direction_vector, env)
        
        drift = np.random.normal(0, self.drift_scale, 2)
        perceived_movement = actual_movement + drift
        
        self.home_vector += perceived_movement
        self.perceived_pos += perceived_movement
        self.energy_used += 0.1
        self.cumulative_drift = np.linalg.norm(self.pos - self.perceived_pos)