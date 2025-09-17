import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
import json

def plot_results(log_files):
    if not os.path.exists('results/plots'):
        os.makedirs('results/plots')
        
    with open('config/simulation_params.json', 'r') as f:
        params = json.load(f)
    env_params = params['environment']
    
    # --- THIS IS THE CORRECTED PART ---
    # Read the list of targets and home position from the config
    targets_list = env_params['targets_list']
    home_pos = env_params['home_pos']
    # --- END OF CORRECTION ---

    all_data = pd.concat(
        [pd.read_csv(f) for f in log_files if os.path.exists(f)], 
        ignore_index=True
    )

    # --- PLOT 1: Trajectories ---
    plt.figure(figsize=(14, 12))
    path_files = glob.glob('results/*_path.csv')
    algorithms = np.unique([os.path.basename(f).split('_')[0] for f in path_files])
    
    plot_order = [algo for algo in ['Baseline', 'AntPathIntegration', 'RodentCognitiveMap', 'BeeSwarmPheromones'] if algo.lower() in [a.lower() for a in algorithms]]
    
    for i, algo in enumerate(plot_order):
        ax = plt.subplot(2, 2, i + 1)
        algo_paths = [f for f in path_files if os.path.basename(f).lower().startswith(algo.lower())]
        for path_file in algo_paths:
            path_df = pd.read_csv(path_file)
            ax.plot(path_df['x'], path_df['y'], alpha=0.5, lw=1)
        
        ax.scatter(home_pos[0], home_pos[1], c='blue', s=100, marker='s', label='Home', zorder=5)
        # Plot all targets from the list
        for idx, target in enumerate(targets_list):
            label = 'Targets' if idx == 0 else None
            ax.scatter(target[0], target[1], c='red', s=150, marker='*', label=label, zorder=5)

        ax.set_title(f"Trajectories: {algo}", fontsize=14)
        ax.set_xlim(0, env_params['size'][0])
        ax.set_ylim(0, env_params['size'][1])
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend()

    plt.tight_layout()
    plt.savefig('results/plots/trajectories.png')
    plt.close()
    print("Saved trajectories.png")


    # --- PLOT 2: Cumulative Drift ---
    drift_data = all_data[all_data['cumulative_drift'] > 0]
    if not drift_data.empty:
        plt.figure(figsize=(12, 7))
        sns.lineplot(data=drift_data, x='step', y='cumulative_drift', hue='algorithm')
        plt.title('Path Integration Error: Cumulative Drift Over Time', fontsize=16)
        plt.ylabel('Distance (Perceived vs. Real Position)', fontsize=12)
        plt.xlabel('Simulation Step', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.savefig('results/plots/cumulative_drift.png')
        plt.close()
        print("Saved cumulative_drift.png")


    # --- PLOT 3: Swarm State Distribution ---
    swarm_data = all_data[all_data['algorithm'] == 'BeeSwarmPheromones']
    if not swarm_data.empty:
        state_counts = swarm_data.groupby(['step', 'state']).size().unstack(fill_value=0)
        plt.figure(figsize=(12, 7))
        state_counts.plot(kind='area', stacked=True, figsize=(12, 7), colormap='viridis')
        plt.title('Bee Swarm Dynamics: Agent States Over Time', fontsize=16)
        plt.ylabel('Number of Agents', fontsize=12)
        plt.xlabel('Simulation Step', fontsize=12)
        plt.legend(title='Agent State')
        plt.grid(True, alpha=0.5)
        plt.savefig('results/plots/swarm_states.png')
        plt.close()
        print("Saved swarm_states.png")
    
    # --- PLOT 4 & 5: ORIGINAL PLOTS ---
    # Distance to Target
    all_data['distance_to_target'] = pd.to_numeric(all_data['distance_to_target'], errors='coerce')
    plt.figure(figsize=(12, 7))
    sns.lineplot(data=all_data, x='step', y='distance_to_target', hue='algorithm', errorbar='sd')
    plt.title('Convergence: Distance to Target Over Time', fontsize=16)
    plt.xlabel('Simulation Step', fontsize=12)
    plt.ylabel('Average Distance to Target', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(title='Algorithm')
    plt.savefig('results/plots/distance_vs_time.png')
    plt.close()
    print("Saved distance_vs_time.png")

    # Energy Consumption
    final_energy = all_data.groupby(['algorithm', 'agent_id'])['energy_used'].max().reset_index()
    avg_final_energy = final_energy.groupby('algorithm')['energy_used'].mean().reset_index()
    plt.figure(figsize=(10, 6))
    sns.barplot(data=avg_final_energy, x='algorithm', y='energy_used', palette='viridis')
    plt.title('Energy Efficiency Comparison', fontsize=16)
    plt.xlabel('Algorithm', fontsize=12)
    plt.ylabel('Average Total Energy Used', fontsize=12)
    plt.xticks(rotation=15)
    plt.savefig('results/plots/energy_comparison.png')
    plt.close()
    print("Saved energy_comparison.png")