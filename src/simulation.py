import pandas as pd
import numpy as np
import os
from tqdm import tqdm

from algorithms.baseline_nav import BaselineNavigator
from algorithms.ant_path_integration import AntPathIntegrator
from algorithms.bee_swarm_pheromones import BeeSwarmAgent
from algorithms.rodent_cognitive_map import RodentCognitiveMapper
from algorithms.potential_field_nav import PotentialFieldNavigator
from algorithms.soft_body_haptic_nav import SoftBodyHapticNavigator
from algorithms.neuromorphic_spiking import NeuromorphicSpikingNavigator
from algorithms.swarm_consensus import SwarmConsensusAgent
from algorithms.deep_rl_nav import DeepRLNavigator
from algorithms.chemotaxis_nav import ArtificialChemotaxisNavigator
from algorithms.cpg_locomotion import CPGNavigator

class Simulation:
    def __init__(self, env, algorithm_type, params, show_progress=True):
        self.env = env
        self.algorithm_type = algorithm_type
        self.params = params
        self.max_steps = params['simulation']['max_steps']
        self.agents = self._create_agents()
        self.log_data = []
        self.step_callback = None
        self.show_progress = show_progress

    def register_step_callback(self, callback):
        """Register a callable that executes after every agent update."""
        self.step_callback = callback

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
        elif self.algorithm_type == "PotentialField":
            params = algo_params['potential_field']
            agents.append(
                PotentialFieldNavigator(
                    0,
                    start_pos,
                    speed=params['speed'],
                    attractive_gain=params['attractive_gain'],
                    repulsive_gain=params['repulsive_gain'],
                    obstacle_influence_radius=params['obstacle_influence_radius'],
                    damping=params['damping'],
                    noise_scale=params['noise_scale'],
                )
            )
        elif self.algorithm_type == "SoftBodyHaptic":
            params = algo_params['soft_body_haptic']
            agents.append(
                SoftBodyHapticNavigator(
                    0,
                    start_pos,
                    speed=params['speed'],
                    sensing_arc=params['sensing_arc'],
                    num_sensors=params['num_sensors'],
                    sensor_range=params['sensor_range'],
                    compliance=params['compliance'],
                    orientation_damping=params['orientation_damping'],
                    contact_bias=params['contact_bias'],
                    exploration_noise=params['exploration_noise'],
                )
            )
        elif self.algorithm_type == "ArtificialChemotaxis":
            params = algo_params['artificial_chemotaxis']
            agents.append(
                ArtificialChemotaxisNavigator(
                    0,
                    start_pos,
                    speed=params['speed'],
                    gradient_gain=params['gradient_gain'],
                    trail_deposit=params['trail_deposit'],
                    noise_scale=params['noise_scale'],
                    desensitisation=params['desensitisation'],
                    gradient_radius=params['gradient_radius'],
                    avoidance_gain=params['avoidance_gain'],
                )
            )
        elif self.algorithm_type == "NeuromorphicSpiking":
            params = algo_params['neuromorphic_spiking']
            agents.append(
                NeuromorphicSpikingNavigator(
                    0,
                    start_pos,
                    speed=params['speed'],
                    num_neurons=params['num_neurons'],
                    membrane_decay=params['membrane_decay'],
                    sensory_weight=params['sensory_weight'],
                    boundary_weight=params['boundary_weight'],
                    inhibition_strength=params['inhibition_strength'],
                    refractory_period=params['refractory_period'],
                    noise_scale=params['noise_scale'],
                )
            )
        elif self.algorithm_type == "CPGNavigator":
            params = algo_params['cpg_navigator']
            agents.append(
                CPGNavigator(
                    0,
                    start_pos,
                    speed=params['speed'],
                    base_frequency=params['base_frequency'],
                    coupling_strength=params['coupling_strength'],
                    sensory_gain=params['sensory_gain'],
                    damping=params['damping'],
                    stride_gain=params['stride_gain'],
                    obstacle_frequency_boost=params['obstacle_frequency_boost'],
                )
            )
        elif self.algorithm_type == "SwarmConsensus":
            params = algo_params['swarm_consensus']
            num_agents = params['num_agents']
            for i in range(num_agents):
                agents.append(
                    SwarmConsensusAgent(
                        i,
                        start_pos,
                        speed=params['speed'],
                        neighborhood_radius=params['neighborhood_radius'],
                        cohesion_gain=params['cohesion_gain'],
                        alignment_gain=params['alignment_gain'],
                        separation_gain=params['separation_gain'],
                        target_gain=params['target_gain'],
                        noise_scale=params['noise_scale'],
                        velocity_damping=params['velocity_damping'],
                    )
                )
        elif self.algorithm_type == "DeepRL":
            params = algo_params['deep_rl']
            agents.append(
                DeepRLNavigator(
                    0,
                    start_pos,
                    speed=params['speed'],
                    hidden_size=params['hidden_size'],
                    learning_rate=params['learning_rate'],
                    gamma=params['gamma'],
                    epsilon=params['epsilon'],
                    epsilon_decay=params['epsilon_decay'],
                )
            )
        return agents


    def run(self):
        print(f"--- Running Simulation: {self.algorithm_type} ---")
        pbar = None
        if self.show_progress:
            pbar = tqdm(total=self.max_steps, desc=f"Simulating {self.algorithm_type}")
        for step in range(self.max_steps):
            self.env.update()
            
            # --- UPDATED COMPLETION LOGIC ---
            tasks_complete = 0
            for agent in self.agents:
                swarm_info = None
                if self.algorithm_type in ("BeeSwarmPheromones", "SwarmConsensus"):
                    swarm_info = {
                        "agents": [
                            {
                                "id": other.agent_id,
                                "pos": other.pos.copy(),
                                "velocity": getattr(other, "velocity", np.zeros(2)),
                            }
                            for other in self.agents
                            if other.agent_id != agent.agent_id
                        ]
                    }
                agent.step(self.env, swarm_info)
                self._log_step(step, agent)
                if self.step_callback:
                    self.step_callback(step, agent, self.env)
                if agent.status == "Tour Complete" or agent.status == "Found Target":
                    tasks_complete += 1
            
            if pbar:
                pbar.update(1)
            # End early if ALL agents have completed their specific tasks
            if tasks_complete == len(self.agents):
                if pbar:
                    pbar.set_description(f"Simulating {self.algorithm_type} (Task Complete)")
                break
        
        if pbar:
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
