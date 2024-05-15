## Overview
The Library of Low-Fidelity Vehicle Models is a collection of vehicle models of three varying fidelities each with Nvidia GPU and CPU versions:
1. 11 Degree Of Freedom (DOF) Model - [CPU](./11dof/), [Nvidia GPU](./11dof-gpu/)
2. 18 Degree Of Freedom (DOF) Model - [CPU](./18dof/), [Nvidia GPU](./18dof-gpu/)
3. 24 Degree Of Freedom (DOF) Model - [CPU](./24dof/), [Nvidia GPU](./24dof-gpu/)

Details about the model can be found in the respective READMEs or in the paper [here](../paper/). Here we discuss the build instructions as well as instructions to run the models.

## Pre-requisites
1. [CMake Version 3.10 or higher](https://cmake.org/download/)
For linux users, you can install CMake using the following command
```bash
sudo apt-get install cmake
```
2. C++ Compiler (GCC/Clang) with C++17 support
For linux users, you can install GCC using the following command
```bash
sudo apt-get install g++
```
3. Python3 (for the Python Wrapper)
4. [SWIG (for the Python Wrapper)](https://www.swig.org/download.html)
For linux users, you can install SWIG using the following command
```bash
sudo apt-get install swig
```
5. [Sundials](https://sundials.readthedocs.io/en/latest/) (Optional - for the Sundials integrator) - See build instructions [here](#optional---sundials)
6. Nvidia GPU along with NVCC and CUDA Version 8.0 or higher (for the GPU versions of models). We recommend installing the [CUDA Toolkit](https://developer.nvidia.com/cuda-12-0-0-download-archive) to meet these dependencies

## Build instructions
A CMake configuration is used for both the C++ model and the Python Wrapper. First, in the CWD (`PATH_TO_REPO/wheeled_vehicle_models/11dof/`) create a folder `build` and move into it
```bash
mkdir build
cd build
```
Assuming you have CMake installed, run 
```bash
ccmake ../
```
It's important to highlight that there are several methods to set the CMake options. The command provided earlier launches a curses-based GUI for CMake, which allows you to configure options interactively. Alternatively, you could use the cmake-gui for a graphical interface, or you can set the CMake options directly from the command line. The method you choose for setting the options is not as crucial as ensuring the correct options are set. Below is a detailed explanation of the available CMake options to help you configure them according to your environment for building the models.

### Setting CMake options
Once you launch the curses-based GUI for CMake, you will see an `Empty Cache`, hit `c` on your keyboard to see the CMake options. Here is the description of the CMake options
- `BUILD_11DOF` - Bool to switch on the building of the CPU version of 11 DOF model. Default: `OFF`
- `BUILD_18DOF` - Bool to switch on the building of the CPU version of 18 DOF model. Default: `OFF`
- `BUILD_24DOF` - Bool to switch on the building of the CPU version of 24 DOF model. Default: `OFF`
- `BUILD_11DOF_GPU` - Bool to switch on the building of the Nvidia GPU version of 11 DOF model. Default: `OFF`
- `BUILD_18DOF_GPU` - Bool to switch on the building of the Nvidia GPU version of 18 DOF model. Default: `OFF`
- `BUILD_24DOF_GPU` - Bool to switch on the building of the Nvidia GPU version of 24 DOF model. Default: `OFF`
- `BUILD_PYTHON_MODULE` - Bool to switch on the building of the Python wrapper. Default: `OFF`
- `CMAKE_BUILD_TYPE` - Set the Build type (`Release` or `Debug`). Press `t` on your keyboard to toggle to `advanced mode` and see what Flags these build types use. Default: `Debug`
- `CMAKE_INSTALL_PREFIX` - Defines the root directory for installation when using the install target. Default (on Linux): `/usr/local`
- `USE_OPENMP` - Option to switch `ON` OpenMP for calculation of system RHS Jacobians. Leads to about 2X speed up. Default: `OFF`
- `USE_SUNDIALS` - Option to link against [Sundials](https://sundials.readthedocs.io/en/latest/) and use the [CVODES](https://sundials.readthedocs.io/en/latest/cvodes/index.html) integrator to solve the system ODE's - Default: `OFF`

Once these options are set, hit `c` to configure. When `USE_SUNDIALS` is set to `ON`, you will be required to provide responses to additional CMake options.

#### Optional - GPU models
If you set the `ON` for the GPU models, you also have the choice to set the following CMake options - we however recommend leaving these in their default state
- `CMAKE_CUDA_ARCHITECTURES` - Set this to the CUDA Architecture in your machine for highly optimized code. See [here](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/) for more details. Default: `52`
- `CUDA_HOST_COMPILER` - Path to CUDA compiler. This can be left empty for now. Default: blank
- `CUDA_SDK_ROOT_DIR` - Path to CUDA SDK. Leave as is. Default: blank
- `CUDA_TOOLKIT_ROOT_DIR` - Path to the CUDA Toolkit. Should be automatically filled in. Default: `path_to_toolkit_on_your_machine`
- `CUDA_USE_STATIC_CUDA_RUNTIME` - Boolean for whether CUDA runtime is statically linked. Leave as is. Default: `ON`
- `CUDA_rt_LIBRARY` - Path to CUDA runtime library. Should be automatically filled in. Default: `path_to_runtime_library_on_your_machine`

#### Optional - Sundials
To use the Sundials integrator, the user must have the Sundials library built and set `USE_SUNDIALS` to `ON`.  
##### Building Sundials
For linux users we provide a build script [here](./build_sundials.sh) which can be used to build Sundials. Just run
```bash
bash buildSundials.sh
```
This will create a sundials build directory in the `PATH_TO_REPO/wheeled_vehicle_models/` called `sundials-build`. When configuring the CMake options, you will need to set the `SUNDIALS_DIR` to the absolute path of this directory.
##### Setting Sundials directory
Once Sundials is built successfully, `USE_SUNDIALS` is set to `ON` and `c` is hit to configure, the following option will appear
- `SUNDIALS_DIR` - Here, set the absolute path to the sundials build directory. This is the directory in which the `SUNDIALSConfig.cmake` file is found.
Once this path is correctly set, hit `c` to configure.

### Generate
Once all the required options are set hit `c` to configure and `g` to generate. This will then populate your `build` directory with the necessary `makefile` with which you can build the 11 DOF model along with its demos by running
```bash
make -j4
```
We recommend to not run `make install` to ensure path consistencies for the demos (as they call upon certain data files relative to the build directory).  
Once this is run, the `build` folder will be populated with the executables of the [demos](./demos) and the Python library (if `BUILD_PYTHON_MODULE` is set to `ON`).  


### Python Wrapper
Since the Python module is built using SWIG, the path to the Python wrapper library needs to be appended to `PYTHON_PATH`. This library can be found in the build directory mentioned above and its path can be set (on Linux) permanently using the following instructions
1) First, open your `bashrc` or `zshrc` file in a text-editor.
2) To this file add the following line
```bash
export PYTHONPATH="${PYTHONPATH}:PATH_TO_REPO/wheeled_vehicle_models/build/"
```
3) Source the `bashrc` or `zshrc` file
```bash
source ~/.zshrc
```
To check if you have correctly built the Python module, run (taking the 11 DOF model as an example)
```python
import pydof11 as dof11
```

### How do I use the models?
The above process will populate the build folder with executables of the demos from each of the models. Each models executables are placed in their respective folders. For example, the 11 DOF model demos are placed in `./build/11dof/`. Many of these demos require command line arguments specifying the input controls for the vehicles. For a description of how to create these control input files, or how to use the default control input files provided, see, for instance for the 11 DOF model, [here](./11dof/README.md#how-do-i-provide-driver-inputs).

#### Example of running a demo
To run any of the executables of the demos, you can run the following command from the build directory providing the necessary command line arguments. To showcase an example of the entire workflow, we provide an example here.

Suppose you would like to run the 18 dof HMMWV vehicle demo with the half-implicit integrator. The source code for this demo can be found [here](./18dof/demos/HMMWV/demo_hmmwv_hi.cpp). At the top of the source code, you will find the instructions for running the demo. In this case, we see that we need to provide an input file with the driver inputs. You can either create this input file from scratch, or use an already existing one. For the 18 DOF model, you can see all the available input files [here](./18dof/data/input/). For this example, we will use the `acc3.txt` file which provides a ramp throttle input for a 10 second simulation. To run this demo, first move to the folder which contains the executable for the demo
```bash
cd build/18dof/
```
Then run the executable with the input file as an argument
```bash
./demo_hmmwv_hi_18 acc3
```
Notice that the input file name is provided without the `.txt` extension. This will run the demo with the provided input file. The demo also outputs a csv file with the vehicle states and some additional information at a user specified frequency in a user specified output directory. For this demo, the frequency is set to 100 Hz and the output file is set to `18dof/data/output/{INPUT_FILE_NAME}.csv`. You can then go to this folder and view the output csv file.

The overall structure of the demos in each of the models is similar. The source code for the demos can be found in the `demos` folder of each model. The input files can be found in the `data/input` folder of each model. The output files are saved in the `data/output` folder of each model.