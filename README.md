# Neuromorphic Navigation Simulation

## Overview

This project is a Python-based simulation environment for testing and comparing bio-inspired navigation algorithms, based on the principles outlined in the research paper "Neuromorphic Navigation Systems for Autonomous Robots." The simulation implements several navigation strategies, from simple baselines to complex cognitive models.

The project has been enhanced to include complex, continuous foraging tasks to test the long-term stability and efficiency of the advanced models. The primary goal is to generate detailed performance data (in CSV format) to analyze the efficiency, accuracy, and robustness of these neuromorphic strategies.

## Author

* **Chandan Sheikder**

## Features

- **Modular Algorithm Design**: Easily add, modify, or compare different navigation algorithms.
- **Configurable Simulations**: Control environment size, obstacles, targets, and algorithm-specific parameters via a JSON config file.
- **Detailed Data Logging**: Outputs detailed CSV logs with per-step data on position, status, state, energy use, and cumulative drift.
- **Advanced Implemented Algorithms**:
    - **Baseline Search**: A simple random-walk algorithm for performance comparison.
    - **Ant Path Integration**: Navigates a multi-target tour and includes a systematic "lost" search behavior to handle accumulated drift.
    - **Bee Swarm Pheromones**: Features a complex multi-agent system with 'Scout' and 'Forager' roles and a recruitment model.
    - **Rodent Cognitive Map**: A sophisticated model with Grid Cells, Place Cells, and Head-Direction Cells, capable of continuous, looped foraging to test long-term navigation.
- **Comprehensive Visualization**: Includes a script to automatically generate multiple plots from the results, including agent trajectories, performance comparisons, and model-specific analytics.

## Project Structure

```
neuromorphic_navigation_project/
|
├── main.py                 # Main script to run simulations
├── README.md               # This file
├── requirements.txt        # Python dependencies
|
├── src/                    # Core simulation logic
│   ├── environment.py      # Simulation environment class
│   ├── robot.py            # Base robot class
│   ├── simulation.py       # Simulation runner and data logger
│   └── utils/
│       └── plotter.py      # Plotting utility
|
├── algorithms/             # Navigation algorithm implementations
│   ├── baseline_nav.py
│   ├── ant_path_integration.py
│   ├── bee_swarm_pheromones.py
│   └── rodent_cognitive_map.py
|
├── config/
│   └── simulation_params.json # Simulation parameters
|
└── results/                # Output directory for CSV logs and plots
```

## Setup and Installation

1.  **Ensure Python 3 is installed.**
2.  **Set up the project directory and files** as described in the structure above.
3.  **Navigate to the project's root directory** in your terminal.
4.  **Create and activate a virtual environment:**
    ```bash
    # Use python3 on macOS/Linux
    python3 -m venv venv
    source venv/bin/activate
    ```
5.  **Install the required dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

## How to Run Simulations

From the project's root directory, with your virtual environment activated, run the main script:

```bash
python3 main.py
```

The script will run all simulations sequentially, save the `.csv` log files into the `results/` directory, and generate all comparison plots in the `results/plots/` subdirectory.

## Understanding the Output

### CSV Log Files

Each simulation produces a CSV file with detailed metrics, including: `step`, `agent_id`, `algorithm`, `x`, `y`, `distance_to_target`, `status`, `state`, `energy_used`, `cumulative_drift`, and `landmarks_discovered`.

### Generated Plots

The `plotter.py` utility will generate five plots:
1.  **Trajectories**: The actual paths taken by the agents.
2.  **Cumulative Drift**: Error accumulation for the Ant and Rodent models.
3.  **Swarm States**: The distribution of Scout vs. Forager agents over time.
4.  **Distance vs. Time**: The convergence rate towards the target.
5.  **Energy Comparison**: A final comparison of the total energy consumed by each algorithm.

## License

This project is licensed under the MIT License.