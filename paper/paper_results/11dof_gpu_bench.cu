// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// See the README.md for more details on how to run the benchmarks
//
// =============================================================================
#include <cuda.h>
#include <iostream>
#include <random>
#include <cuda_runtime.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "dof11_halfImplicit_gpu.cuh"

using namespace d11GPU;
int main(int argc, char** argv) {
    // Get total number of vehicles from command line
    unsigned int num_vehicles = std::stoul(argv[1]);
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[2]);

    std::string driver_file = "../../../wheeled_vehicle_models/18dof/data/input/acc3.txt";

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = (char*)"../../../wheeled_vehicle_models/11dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../../wheeled_vehicle_models/11dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles, driver_file);

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

    solver.SetEndTime(10.0);

    // Solve
    // Time the solve
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);
    solver.Solve();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Number of vehicles: " << num_vehicles << "\n";
    std::cout << "Solve time (ms): " << milliseconds << "\n";
}
