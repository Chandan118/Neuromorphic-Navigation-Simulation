import numpy as np
from src.robot import Robot

class RodentCognitiveMapper(Robot):
    """
    Navigator with a cognitive map, updated for continuous, looped foraging.
    After returning home, it will start the tour again.
    """
    def __init__(self, agent_id, start_pos, speed=1.0, grid_cell_drift=0.01, place_cell_radius=5, head_direction_drift=0.02):
        super().__init__(agent_id, start_pos, "RodentCognitiveMap", speed)
        self.grid_cell_pos = np.array(start_pos, dtype=float)
        self.grid_cell_drift_scale = grid_cell_drift
        self.place_cell_radius = place_cell_radius
        self.known_landmarks = {}
        self.head_direction = self.orientation
        self.head_direction_drift_scale = head_direction_drift
        self.max_turn_rate = np.pi / 8
        self.target_idx = 0
        self.state = "Foraging" # Foraging -> ReturningHome -> loop back to Foraging

    def step(self, env, swarm_info=None):
        # The simulation for this agent will now run until max_steps is reached
            
        self.detect_landmarks(env)
        
        if self.known_landmarks:
            errors = []
            for lm_id, lm_pos in self.known_landmarks.items():
                if np.linalg.norm(self.pos - lm_pos) < self.place_cell_radius:
                    errors.append(lm_pos - self.grid_cell_pos)
            if errors:
                avg_error = np.mean(errors, axis=0)
                self.grid_cell_pos += 0.2 * avg_error
                self.energy_used += 0.5

        # Determine current navigation goal
        goal_pos = None
        if self.state == "Foraging":
            goal_pos = env.targets_list[self.target_idx]
            if np.linalg.norm(self.pos - goal_pos) < 3.0:
                self.target_idx += 1
                if self.target_idx >= len(env.targets_list):
                    self.state = "ReturningHome"
        
        elif self.state == "ReturningHome":
            goal_pos = env.home_pos
            # --- THIS IS THE UPDATED LOGIC ---
            # When it reaches home, instead of stopping, it resets for another tour.
            if np.linalg.norm(self.pos - goal_pos) < 3.0:
                self.state = "Foraging"
                self.target_idx = 0
                # Add an "unloading" energy cost and continue
                self.energy_used += 10
                # Set goal to the first target for the new tour
                goal_pos = env.targets_list[self.target_idx]
            # --- END OF UPDATE ---

        # Navigation with Head Direction
        target_direction_vector = goal_pos - self.grid_cell_pos
        desired_angle = np.arctan2(target_direction_vector[1], target_direction_vector[0])
        
        angle_diff = (desired_angle - self.head_direction + np.pi) % (2 * np.pi) - np.pi
        turn_angle = np.clip(angle_diff, -self.max_turn_rate, self.max_turn_rate)
        self.head_direction += turn_angle + np.random.normal(0, self.head_direction_drift_scale)
        self.energy_used += abs(turn_angle) * 0.2

        movement_vector = np.array([np.cos(self.head_direction), np.sin(self.head_direction)])
        self.move_and_update_grid_cells(movement_vector, env)
        
        self.cumulative_drift = np.linalg.norm(self.pos - self.grid_cell_pos)
        self.landmarks_discovered = len(self.known_landmarks)

    def detect_landmarks(self, env):
        for i, obs_pos in enumerate(env.obstacles):
            if np.linalg.norm(self.pos - obs_pos) < self.place_cell_radius:
                if i not in self.known_landmarks:
                    self.known_landmarks[i] = np.array(obs_pos)

    def move_and_update_grid_cells(self, direction_vector, env):
        actual_movement = direction_vector * self.speed
        super().move(direction_vector, env)
        
        drift = np.random.normal(0, self.grid_cell_drift_scale, 2)
        self.grid_cell_pos += actual_movement + drift
        self.energy_used += 0.3