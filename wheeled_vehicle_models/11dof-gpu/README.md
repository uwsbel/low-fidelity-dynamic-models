## Overview
The 11 DOF GPU model is theoretically exactly the same as the CPU model, in that it solves the same equations of motions. However, the GPU version of the model has solvers that support solving multiple sets of these equations of motions on different GPU threads parallely. The GPU models however have a more limited functionallity when compared to the CPU models. For instance:
- The GPU models only support the Half-Implicit integrator
- Because Sundials is not supported, the GPU models do not support Forward Sensitivity Analysis (FSA)
- The GPU models do not provide system RHS Jacobians
- The GPU models are not wrapped to Python
However, the GPU model can simultaneously simulate upto 300,000 vehicles in real-time at a time step of $1e-3$ when benchmarked on an Nvidia A100 GPU. See chapter 6 [here](https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc) for more details.
The model parameters and inputs are provided in the same way as the CPU model. The repository also contains [demos](./demos/) that describe how the GPU models can be used.

## Build instructions for the 11 DOF GPU Model Demos
### Prerequisites
Ensure that you have an Nvidia GPU and have a CUDA V8.0> compiler installed. To install a cuda compiler please refer to the official Nvidia documentation [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html).

Once you have a working CUDA compiler, in the CWD (`PATH_TO_REPO/wheeled_vehicle_models/11dof-gpu/`) create a folder `build` and move into it
```bash
mkdir build
cd build
```
Assuming you have CMake installed, run 
```bash
ccmake ../
```
It's important to highlight that there are several methods to set the CMake options. The command provided earlier launches a curses-based GUI for CMake, which allows you to configure options interactively. Alternatively, you could use the cmake-gui for a graphical interface, or you can set the CMake options directly from the command line. The method you choose for setting the options is not as crucial as ensuring the correct options are set. Below is a detailed explanation of the available CMake options to help you configure them according to your environment for building the model.
### Setting CMake Options
Once you launch the curses-based GUI for CMake, you will see an `Empty Cache`, hit `c` on your keyboard to see the CMake options. Here is the description of the CMake options
- `CMAKE_BUILD_TYPE` - Set the Build type (`Release` or `Debug`). Press `t` on your keyboard to toggle to `advanced mode` and see what Flags these build types use. Default: `Debug`
- `CMAKE_CUDA_ARCHITECTURES` - Set this to the CUDA Architecture in your machine for highly optimized code. See [here](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/) for more details. Default: `52`
- `CMAKE_INSTALL_PREFIX` - Defines the root directory for installation when using the install target. Default (on Linux): `/usr/local`
- `CUDA_HOST_COMPILER` - Path to CUDA compiler. This can be left empty for now. Default: blank
- `CUDA_SDK_ROOT_DIR` - Path to CUDA SDK. Leave as is. Default: blank
- `CUDA_TOOLKIT_ROOT_DIR` - Path to the CUDA Toolkit. Should be automatically filled in. Default: `path_to_toolkit_on_your_machine`
- `CUDA_USE_STATIC_CUDA_RUNTIME` - Boolean for whether CUDA runtime is statically linked. Leave as is. Default: `ON`
- `CUDA_rt_LIBRARY` - Path to CUDA runtime library. Should be automatically filled in. Default: `path_to_runtime_library_on_your_machine`

 Once the options are set, hit `c` to configure and `g` to generate. This will then populate your `build` directory with the necessary `makefile` with which you can build the 11 DOF GPU model along with its demos by running
```bash
make -j4
```
We recommend to not run `make install` to ensure path consistencies for the demos (as they call upon certain data files relative to the build directory). 
 
 
