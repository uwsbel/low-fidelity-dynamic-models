# Fast and Accurate Low Fidelity Dynamic Models for Robotics

## Overview
This repository hosts a collection of low fidelity dynamic models, optimized for speed and efficiency, and primarily designed for robotics applications. It features models for wheeled robots, including vehicles, and is equipped with integrators (both half-implicit and implicit, utilizing Sundials) for simulating these models from a given initial state.

Using CMake, the user can choose to build the models to execute on the CPU or NVIDIA GPU cards. The CPU models are implemented in C++, whereas the GPU models utilize CUDA. A Python API is also available, provided through SWIG-wrapped C++ models.

### Key Features
1. **High-Speed Performance**: Models surpass real-time processing speeds. For instance, the 18 Degrees of Freedom (DOF) model achieves 2000x faster performance than real-time on standard CPUs, with an integration timestep of `1e-3` s.
   
2. **GPU Optimization for Scalability**: The GPU models are adept at parallel simulations of multiple vehicles. The 18 DOF GPU model, for example, can simulate 300,000 vehicles in real-time on an NVIDIA A100 GPU.

3. **Python API**: The SWIG-wrapped Python version maintains significant speed, being only 8 times slower than the C++ models, thereby offering Python's ease of use with C++ efficiency.

4. **Advanced Analysis with Sundials**: The CPU models support Forward Sensitivity Analysis (FSA) for select parameters. The use of a half-implicit integrator allows easy access to Jacobians of the system's RHS in relation to states and controls, beneficial for gradient-based Model Predictive Control (MPC) methods.

5. **Comprehensive Vehicle Dynamics Simulation**: Including models for the engine, powertrain, and torque converter, these simulations closely replicate actual vehicles. Users also have a choice between two semi-empirical TMeasy tire models, noted for their accuracy and performance at high vehicle speeds.

6. **User-Friendly Configuration**: Parameters for the models can be set dynamically at runtime through JSON files.

This library seeks to establish the right balance between speed and accuracy in robotics simulation, especially in the context of vehicle dynamics.

## Repository Structure

Several sub-folders contain each the source code for a vehicle model and the numerical integrator (time stepper) that runs it. CMake configurations have been set up to help with building _demos_, enabling the user to quickly kick off a simulation. The main repo folders are as follows:

- **docs**: Documentation related to the dynamic models and their usage. See [README](./docs/README.md) for instructions to build the docs locally.
- **wheeled_vehicle_models**: Contains subdirectories for different DOF models for wheeled vehicles:
  - **11dof**: CPU version for the 11 DOF wheeled vehicle model. See [README](./wheeled_vehicle_models/11dof/README.md) for build and use instructions.
  - **11dof-gpu**: GPU version for the 11 DOF wheeled vehicle model. See [README](./wheeled_vehicle_models/11dof-gpu/README.md) for build and use instructions.
  - **18dof**: CPU version for the 18 DOF wheeled vehicle model. See [README](./wheeled_vehicle_models/18dof/README.md) for build and use instructions.
  - **18dof-gpu**: GPU version for the 18 DOF wheeled vehicle model. See [README](./wheeled_vehicle_models/18dof-gpu/README.md) for build and use instructions.
  - **24dof**: CPU version for the 24 DOF wheeled vehicle model. See [README](./wheeled_vehicle_models/24dof/README.md) for build and use instructions.
  - **24dof-gpu**: GPU version for the 24 DOF wheeled vehicle model. See [README](./wheeled_vehicle_models/24dof-gpu/README.md) for build and use instructions.
  - **third_party/rapidjson**: Third-party libraries and dependencies, specifically `rapidjson` for JSON handling.
  - **utils**: Utility scripts and tools supporting the dynamic models.
Each vehicle model has its own (although very similar) CMake configuration and build procedure. README files are available in each model sub-directory describing this procedure. 


## If you are interested to contribute, see [Contribution Guidelines](docs/CONTRIBUTING.md)
