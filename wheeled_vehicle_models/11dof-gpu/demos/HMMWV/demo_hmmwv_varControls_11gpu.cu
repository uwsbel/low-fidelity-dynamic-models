// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating 1000 HMMWVs (specified with JSON files), 500 operating on one driver input file
// and the rest operating on another driver input file. Since the Half Implicit solver is the only one supported
// for the GPU models, that is what is used here.
// Use ./executable_name <threads_per_block>
//
// =============================================================================
#include <cuda.h>
#include <iostream>
#include <random>
#include <filesystem>
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
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <threads_per_block>" << std::endl;
        return 1;
    }

    // Set the total number of vehicles
    unsigned int num_vehicles = 1000;
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[1]);

    // Get two driver files
    std::string file_name_1 = "acc3";
    std::string driver_file_1 = "../../11dof-gpu/data/input/" + file_name_1 + ".txt";

    std::string file_name_2 = "double_lane4";
    std::string driver_file_2 = "../../11dof-gpu/data/input/" + file_name_2 + ".txt";

    // Check if driver files exist
    if (!fs::exists(driver_file_1) || !fs::exists(driver_file_2)) {
        std::cerr << "Error: One or both driver files do not exist." << std::endl;
        return 1;
    }

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = "../../11dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../11dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicitGPU solver(num_vehicles);

    // First construct half the vehicles for driver file 1
    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_1);
    // Then construct the other half for driver file 2
    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_2);

    // Set the threads per block
    solver.SetThreadsPerBlock(threads_per_block);

    // Set the time step of the solver
    solver.SetTimeStep(1e-3);

    // Now we initialize the states -> These are all set to 0 (struct initializer)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;
    // Again we initialize the same states for all vehicles
    solver.Initialize(veh_st, tiref_st, tirer_st, num_vehicles);

    // Set the simulation end time
    solver.SetEndTime(22.0);

    // Solve
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);
    solver.Solve();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Solve time (ms): " << milliseconds << "\n";

    // Extract terminal state of chosen vehicles and print the position
    SimState sim_state_1 = solver.GetSimState(499);
    SimState sim_state_2 = solver.GetSimState(999);

    std::cout << "X Position of vehicle 499: " << sim_state_1._veh_state._x << std::endl;
    std::cout << "X Position of vehicle 999: " << sim_state_2._veh_state._x << std::endl;

    return 0;
}
