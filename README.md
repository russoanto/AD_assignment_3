# Particle Filter Localization for Forklift Simulation

## Assignment #4 - Particle Filter Localization

### Objectives

The main objective of this project is to localize a forklift in a simulation using a particle filter. The forklift is equipped with a LiDAR and moves in an environment with known landmarks. The particle filter will be used to estimate the position of the forklift throughout the entire simulation.

### Implemented Features

- **Complete Particle Filter**: The filter runs throughout the entire simulation, covering all main steps:
  - Particle initialization
  - State prediction
  - Update based on measurements
  - Particle resampling

- **Random Particle Initialization**: Particles are initialized randomly within the simulation space.

- **Custom Resampling Method**: A custom resampling method has been implemented based on the paper "[Resampling methods for particle filtering: Classification, implementation and strategies](https://bisite.usal.es/archivos/resampling_methods_for_particle_filtering_classification_implementation_and_strategies.pdf)". This improvement aims to optimize the efficiency of resampling and the quality of the estimate.

- **Performance Optimization**: Several improvements have been made to the code to make the particle filter more efficient in terms of execution and localization accuracy.

### Code Structure

The project includes the following main components:

- `particle_filter.py`: Implements the particle filter with all steps (initialization, prediction, update, resampling).
- `main.py`: Runs the simulation, executes the particle filter, and saves the results.
- `plotter.py`: Script to visualize the estimated trajectories against the ground truth data.
- `resampling.py`: Contains the custom resampling implementation.

### Report

The report describes the performance of the particle filter in different scenarios, analyzing the estimated trajectory, accuracy compared to ground truth, and execution times. The output file `res.txt` contains the X, Y coordinate estimates of the best particle, the corresponding ground truth, and the execution time.

Results are discussed across different scenarios, varying:
- Number of particles
- Sensor errors (LiDAR noise)
- Motion model

### Running Instructions

1. **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

2. **Run the simulation**:
    ```bash
    python main.py
    ```

3. **Visualize the results**:
    Once the simulation is complete, use the following command to generate the trajectory plots:
    ```bash
    python plotter.py
    ```

4. **Output**:
    - The `res.txt` file contains position estimates (X, Y) and execution time.
    - The generated plots show the particle filter’s estimated trajectory compared to the forklift’s actual trajectory.

### Tested Scenarios

The full report includes analysis of the following scenarios:

1. **Scenario 1: Reduced Number of Particles**: Basic configuration with a limited number of particles and low sensor noise.
2. **Scenario 2: High Sensor Noise**: Increased LiDAR noise to test the robustness of the filter.
3. **Scenario 3: Increased Number of Particles**: More particles used to observe the impact on accuracy and execution times.

### Future Improvements

- Further optimization of the resampling algorithm for more complex scenarios.
- Implementation of a more realistic motion model for the forklift.
- Use of parallelization techniques to speed up the particle filter’s execution.

### Author

Project developed for the **Autonumus Driving** course.

For more information, contact: [russoantonio451@gmail.com](mailto:russoantonio451@gmail.com).
