## Overview
The 11 Degree Of Freedom (DOF) vehicle model is a single-track vehicle model that is also often commonly referred to as the 'bicycle' model. This is the simplest (in terms of dynamics) of the vehicle models available in this library and thus also runs the fastest. The Chassis includes 3 DOF at the vehicle lumped Centre of Mass (C.M), representing the vehicle’s yaw, lateral, and longitudinal motions. This model assumes the same engine, torque converter, and powertrain as the 18 DOF and 24 DOF models. However, as it is a single-track model, it uses only two TMeasy tires. Similarly, the driveline now only consists of the center differential, which splits the torque between the front and rear wheels
based on the shaft speeds. The steering mechanism remains unchanged, with only one front wheel that is steered. The 11 DOF model takes the same input as the 18 DOF and 24 DOF models - Throttle $\in [0,1]$, Braking $\in [0,1]$ and Steering $\in [-1,1]$ where $-1$ is a full left turn. See chapter 2 [here](https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc) for more details.

### Where do I set model parameters?
Model parameters are set through JSON files which are then used in the `construct` solver function. Examples of these JSON files are available for 2 different vehicles in [data/json/HMMWV](./data/json/HMMWV) and [data/json/Sedan](./data/json/Sedan). Examples of use are available in [demos](./demos/).

### How do I provide driver-inputs?
Driver Inputs can be provided at run-time during the simualtion or before the beginning of the simulation. For examples on provided driver inputs at run-time, see [demos/HMMWV/demo_hmmwv_hi_step.cpp](./demos/HMMWV/demo_hmmwv_hi_step.cpp).  
These inputs can also be provided before the simulation begins through text files. See [data/input/acc.txt](./data/input/acc.txt) for an example. Here, the first column is the time, second column is steering, third column is throttle and the last column is braking. The solver then computes the steering, throttle and braking inputs at a given time through linear-interpolation. See [demos/HMMWV/demo_hmmwv_hi](./demos/HMMWV/demo_hmmwv_hi.cpp) for more details.

## Build instructions for the 11 DOF CPU Model Demos
A CMake configuration is used for both the C++ model and the Python Wrapper. First, in the CWD (`PATH_TO_REPO/wheeled_vehicle_models/11dof/`) create a folder `build` and move into it
```bash
mkdir build
cd build
```
Assuming you have CMake installed, run 
```bash
ccmake ../
```
It's important to highlight that there are several methods to set the CMake options. The command provided earlier launches a curses-based GUI for CMake, which allows you to configure options interactively. Alternatively, you could use the cmake-gui for a graphical interface, or you can set the CMake options directly from the command line. The method you choose for setting the options is not as crucial as ensuring the correct options are set. Below is a detailed explanation of the available CMake options to help you configure them according to your environment for building the model.

### Setting CMake options
Once you launch the curses-based GUI for CMake, you will see an `Empty Cache`, hit `c` on your keyboard to see the CMake options. Here is the description of the CMake options
- `BUILD_PYTHON_MODULE` - Bool to switch on the building of the Python wrapper. Default: `OFF`
- `CMAKE_BUILD_TYPE` - Set the Build type (`Release` or `Debug`). Press `t` on your keyboard to toggle to `advanced mode` and see what Flags these build types use. Default: `Debug`
- `CMAKE_INSTALL_PREFIX` - Defines the root directory for installation when using the install target. Default (on Linux): `/usr/local`
- `USE_OPENMP` - Option to switch `ON` OpenMP for calculation of system RHS Jacobians. Leads to about 2X speed up. Default: `OFF`
- `USE_SUNDIALS` - Option to link against [Sundials](https://sundials.readthedocs.io/en/latest/) and use the [CVODES](https://sundials.readthedocs.io/en/latest/cvodes/index.html) integrator to solve the system ODE's - Default: `OFF`

Once these options are set, hit `c` to configure. When `USE_SUNDIALS` is set to `ON`, you will be required to provide responses to additional CMake options.

#### Optional - Sundials
To use the Sundials integrator, the user must have the Sundials library built and set `USE_SUNDIALS` to `ON`.  
##### Building Sundials
To Build Sundials, please refer to the [official Sundials installation instructions](https://sundials.readthedocs.io/en/latest/Install_link.html). Ensure that you set options `BUILD_CVODE` and `BUILD_CVODES` to `ON`.
##### Setting Sundials directory
Once Sundials is built successfully, `USE_SUNDIALS` is set to `ON` and `c` is hit to configure, the following option will appear
- `SUNDIALS_DIR` - Here, set the absolute path to the sundials build directory. This is the directory in which the `SUNDIALSConfig.cmake` file is found.
Once this path is correctly set, hit `c` to configure.

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
export PYTHONPATH="${PYTHONPATH}:PATH_TO_REPO/wheeled_vehicle_models/11dof/build/"
```
3) Source the `bashrc` or `zshrc` file
```bash
source ~/.zshrc
```
To check if you have correctly built the Python module, run 
```python
import pydof11 as dof11
```
