// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user provided number of HMMWVs (specified with JSON files), all operating on the
// same driver inputs on the GPU. Since the Half Implicit solver is the only one supported for the GPU models,
// that is what is used here. The structure of the API is very similar to the CPU version except for the
// additional requirements of specifying the number of vehicles and threads per block.
// Use ./executable_name <total_number_of_vehicles> <threads_per_block>
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
#include <filesystem>

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

    std::string file_name = "double_lane4";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file = "../../11dof-gpu/data/input/" + file_name + ".txt";

    // Check if input file exists
    if (!fs::exists(driver_file)) {
        std::cerr << "Error: Driver input file does not exist: " << driver_file << std::endl;
        return 1;
    }

    // Output directory
    std::string outputBasePath = "../../11dof-gpu/data/output/";
    // Ensure output directory exists
    if (!fs::exists(outputBasePath)) {
        fs::create_directories(outputBasePath);
    }

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = "../../11dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../11dof-gpu/data/json/HMMWV/tmeasy.json";

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

    // Enable output for a subset of the vehicles
    std::string outputFileName = outputBasePath + file_name + "_hmmwv11";
    solver.SetOutput(outputFileName, 100, false, 50);

    // Set the simulation end time
    solver.SetEndTime(22.0);

    // Time the solve
    auto start = std::chrono::high_resolution_clock::now();
    solver.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Solve time: " << elapsed.count() << " s\n";

    return 0;
}
