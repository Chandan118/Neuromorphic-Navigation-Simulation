import pandas as pd
import numpy as np
import os
from tqdm import tqdm

from algorithms.baseline_nav import BaselineNavigator
from algorithms.ant_path_integration import AntPathIntegrator
from algorithms.bee_swarm_pheromones import BeeSwarmAgent
from algorithms.rodent_cognitive_map import RodentCognitiveMapper

class Simulation:
    def __init__(self, env, algorithm_type, params):
        self.env = env
        self.algorithm_type = algorithm_type
        self.params = params
        self.max_steps = params['simulation']['max_steps']
        self.agents = self._create_agents()
        self.log_data = []

    def _create_agents(self):
        # ... (This function remains unchanged from the previous version) ...
        agents = []
        start_pos = self.env.home_pos
        sim_params = self.params['simulation']
        algo_params = self.params['algorithms']

        if self.algorithm_type == "Baseline":
            agents.append(BaselineNavigator(0, start_pos))
        elif self.algorithm_type == "AntPathIntegration":
            params = algo_params['ant_path_integration']
            agents.append(AntPathIntegrator(0, start_pos, speed=params['speed'], drift_scale=params['drift_scale'], search_spiral_rate=params['search_spiral_rate']))
        elif self.algorithm_type == "RodentCognitiveMap":
            params = algo_params['rodent_cognitive_map']
            agents.append(RodentCognitiveMapper(0, start_pos, speed=params['speed'], grid_cell_drift=params['grid_cell_drift'], place_cell_radius=params['place_cell_radius'], head_direction_drift=params['head_direction_drift']))
        elif self.algorithm_type == "BeeSwarmPheromones":
            num_agents = sim_params['num_swarm_agents']
            params = algo_params['bee_swarm']
            num_scouts = int(num_agents * params['num_scouts_ratio'])
            for i in range(num_agents):
                is_scout = i < num_scouts
                agents.append(BeeSwarmAgent(i, start_pos, speed=params['speed'], deposit_rate=params['pheromone_deposit_rate'], is_scout=is_scout))
        return agents


    def run(self):
        print(f"--- Running Simulation: {self.algorithm_type} ---")
        pbar = tqdm(total=self.max_steps, desc=f"Simulating {self.algorithm_type}")
        for step in range(self.max_steps):
            self.env.update()
            
            # --- UPDATED COMPLETION LOGIC ---
            tasks_complete = 0
            for agent in self.agents:
                agent.step(self.env)
                self._log_step(step, agent)
                if agent.status == "Tour Complete" or agent.status == "Found Target":
                    tasks_complete += 1
            
            pbar.update(1)
            # End early if ALL agents have completed their specific tasks
            if tasks_complete == len(self.agents):
                pbar.set_description(f"Simulating {self.algorithm_type} (Task Complete)")
                break
        
        pbar.close()
        self.save_log()

    def _log_step(self, step, agent):
        # ... (This function remains unchanged) ...
        self.log_data.append({
            "step": step,
            "agent_id": agent.agent_id,
            "algorithm": agent.algorithm,
            "x": agent.pos[0],
            "y": agent.pos[1],
            "distance_to_target": agent.distance_to_target,
            "status": agent.status,
            "state": agent.state,
            "energy_used": agent.energy_used,
            "cumulative_drift": agent.cumulative_drift,
            "landmarks_discovered": agent.landmarks_discovered
        })

    def save_log(self):
        # ... (This function remains unchanged) ...
        if not os.path.exists('results'):
            os.makedirs('results')
        
        df = pd.DataFrame(self.log_data)
        # Save agent paths for trajectory plotting
        if len(self.agents) < 10: # Only save paths for small numbers of agents
            for agent in self.agents:
                path_df = pd.DataFrame(agent.path, columns=['x', 'y'])
                path_df.to_csv(f"results/{agent.algorithm}_{agent.agent_id}_path.csv", index=False)

        filename = f"results/{self.algorithm_type.lower()}_log.csv"
        df.to_csv(filename, index=False)
        print(f"Log saved to {filename}")