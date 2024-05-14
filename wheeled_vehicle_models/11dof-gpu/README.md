## Overview
The 11 DOF GPU model is theoretically exactly the same as the CPU model, in that it solves the same equations of motions. However, the GPU version of the model has solvers that support solving multiple sets of these equations of motions on different GPU threads parallely. The GPU models however have a more limited functionallity when compared to the CPU models. For instance:
- The GPU models only support the Half-Implicit integrator
- Because Sundials is not supported, the GPU models do not support Forward Sensitivity Analysis (FSA)
- The GPU models do not provide system RHS Jacobians
- The GPU models are not wrapped to Python
However, the GPU model can simultaneously simulate upto 300,000 vehicles in real-time at a time step of $1e-3$ when benchmarked on an Nvidia A100 GPU. See chapter 6 [here](https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc) for more details.
The model parameters and inputs are provided in the same way as the CPU model. The repository also contains [demos](./demos/) that describe how the GPU models can be used.

## How do I run the demos?
See [here](../README.md#how-do-i-use-the-models) for a general description of how to run the demos. For the 11 DOF model, the demos are placed in the [demos](./demos) folder. The demos are built in the [build](../README.md#generate) folder. The demos require command line arguments specifying the input controls for the vehicles. Additionally, some of the GPU demos require information about the number of vehicles desired to be simulated and the number of threads per block to be launched. We recommend using `32` for the number of threads per block. For a description of how to create these control input files, or how to use the default control input files provided, see [here](../11dof/README.md#how-do-i-provide-driver-inputs).
 
 
