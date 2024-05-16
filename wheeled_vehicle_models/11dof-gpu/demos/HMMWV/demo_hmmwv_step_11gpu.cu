// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user provided number of HMMWVs (specified with JSON files) on a step-by-step
// basis. The "step" at which a new control input is provided can be set with SetKernelSimTime as shown here.
// Since the Half Implicit solver is the only one supported for the GPU models, that is what is used here.
// When the solver is used in a step-by-step manner, the output is not stored in a file (unlike the CPU models).
// However, access to the vehicle states every control time step is provided through the GetSimState function.
// Use ./executable_name <total_number_of_vehicles> <threads_per_block>
//
// =============================================================================
#include <cuda.h>
#include <iostream>
#include <filesystem>
#include <random>
#include <cuda_runtime.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "dof11_halfImplicit_gpu.cuh"

namespace fs = std::filesystem;
using namespace d11GPU;

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <total_number_of_vehicles> <threads_per_block>" << std::endl;
        return 1;
    }

    // Convert command line arguments
    unsigned int num_vehicles = std::stoul(argv[1]);
    unsigned int threads_per_block = std::stoul(argv[2]);

    std::string file_name = "acc3";
    std::string inputPath = "../../11dof-gpu/data/input/" + file_name + ".txt";

    // Check if input file exists
    if (!fs::exists(inputPath)) {
        std::cerr << "Error: Input file does not exist: " << inputPath << std::endl;
        return 1;
    }

    // Vehicle specification
    std::string vehParamsJSON = "../../11dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../11dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles);

    // Set the threads per block
    solver.SetThreadsPerBlock(threads_per_block);

    // Set the time step of the solver
    solver.SetTimeStep(1e-3);

    // Set the "step" time step
    double control_time_step = 1e-1;
    solver.SetKernelSimTime(control_time_step);

    // Initialize the states for all vehicles
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;
    solver.Initialize(veh_st, tiref_st, tirer_st, num_vehicles);

    double endTime = 10.0;
    double timeStep = solver.GetStep();
    double t = 0;
    double new_time = 0;
    double throttle;
    double steering;
    double braking;

    // Time the solve process
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);

    while (t < (endTime - timeStep / 10.)) {
        if (t > 1) {
            throttle = 0.5;
            steering = 0.0;
            braking = 0.0;
        } else {
            throttle = 0.0;
            steering = 0.0;
            braking = 0.0;
        }

        new_time = solver.SolveStep(t, steering, throttle, braking);  // Solve for the current time
        t = new_time;
    }

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Solve time (ms): " << milliseconds << "\n";

    // Extract terminal state of chosen vehicles and print the position
    SimState sim_state_1 = solver.GetSimState(0);
    std::cout << "X Position of vehicle 1: " << sim_state_1._veh_state._x << std::endl;

    return 0;
}
