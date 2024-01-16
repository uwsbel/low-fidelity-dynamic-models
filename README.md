# Fast and Accurate Low Fidelity Dynamic Models for Robotics

## Overview
This repository hosts a comprehensive collection of low fidelity dynamic models, optimized for speed and efficiency, and primarily designed for robotics applications. It features models for wheeled robots, including vehicles, and is equipped with integrators (both half-implicit and implicit, utilizing Sundials) for simulating these models from a given initial state.

The models are compatible with both CPU and Nvidia GPU platforms, thanks to a standard CMake configuration. The CPU models are developed in C++, whereas the GPU models utilize CUDA. A Python API is also available, provided through SWIG-wrapped C++ models.

### Key Advantages
1. **High-Speed Performance**: Models greatly surpass real-time processing speeds. For instance, the 18 Degrees of Freedom (DOF) model achieves 2000x faster performance than real-time on standard CPUs, with an integration timestep of `1e-3`.
   
2. **GPU Optimization for Scalability**: The GPU models are adept at parallel simulations of multiple vehicles. The 18 DOF GPU model, for example, can simulate 300,000 vehicles in real-time on an Nvidia A100 GPU.

3. **Python API for Convenience**: The SWIG-wrapped Python version maintains significant speed, being only 8 times slower than the C++ models, thereby offering Python's ease of use with C++ efficiency.

4. **Advanced Analysis with Sundials**: The CPU models support Forward Sensitivity Analysis (FSA) for select parameters. The use of a half-implicit integrator allows easy access to Jacobians of the system's RHS in relation to states and controls, beneficial for gradient-based Model Predictive Control (MPC) methods.

5. **Comprehensive Vehicle Dynamics Simulation**: Including models for the engine, powertrain, and torque converter, these simulations closely replicate actual vehicles. Users also have a choice between two semi-empirical TMeasy tire models, noted for their accuracy and performance at high vehicle speeds.

6. **User-Friendly Configuration**: Parameters for the models can be set dynamically at runtime through JSON files, ensuring a seamless user experience.

This library is thus an ideal resource for those in need of a balance between speed and accuracy in robotics simulations, especially in the context of vehicle dynamics.

## Repository Structure

The repository is set up with multiple directories, each with a specific role. You'll find sub-folders that contain the source code for a range of vehicle models and the integrators that run them. We've provided CMake configurations to help you build _demos_, so you can see how to get the models and integrators working together to kick off simulations. Here’s a quick outline of the main folders:

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
