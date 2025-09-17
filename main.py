import json
import os
from src.environment import Environment
from src.simulation import Simulation
from src.utils.plotter import plot_results

def main():
    """
    Loads configuration, runs all simulations, and generates plots.
    """
    # Load parameters
    with open('config/simulation_params.json', 'r') as f:
        params = json.load(f)

    env_params = params['environment']
    algo_params = params['algorithms']

    # List of algorithms to simulate
    algorithms_to_run = [
        "Baseline",
        "AntPathIntegration",
        "RodentCognitiveMap",
        "BeeSwarmPheromones"
    ]

    log_files = []

    for algo_type in algorithms_to_run:
        # --- THIS IS THE CORRECTED PART ---
        # Create a fresh environment for each simulation using 'targets_list'
        env = Environment(
            size=tuple(env_params['size']),
            num_obstacles=env_params['num_obstacles'],
            targets_list=env_params['targets_list'],
            home_pos=tuple(env_params['home_pos']),
            pheromone_decay_rate=algo_params['bee_swarm']['pheromone_decay_rate']
        )
        # --- END OF CORRECTION ---
        
        sim = Simulation(env, algo_type, params)
        sim.run()
        log_files.append(f"results/{algo_type.lower()}_log.csv")

    # Generate plots from all simulation logs
    print("\n--- Generating Comparison Plots ---")
    if log_files:
        plot_results(log_files)
    else:
        print("No log files found to plot.")

if __name__ == "__main__":
    main()