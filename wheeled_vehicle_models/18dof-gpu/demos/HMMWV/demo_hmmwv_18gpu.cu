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
#include <filesystem>
#include <random>
#include <cuda_runtime.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "dof18_halfImplicit_gpu.cuh"

namespace fs = std::filesystem;
using namespace d18GPU;

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <total_number_of_vehicles> <threads_per_block>" << std::endl;
        return 1;
    }

    // Parse command line arguments
    unsigned int num_vehicles = std::stoul(argv[1]);
    unsigned int threads_per_block = std::stoul(argv[2]);

    std::string file_name = "acc3";
    std::string inputBasePath = "../../18dof-gpu/data/input/";
    std::string outputBasePath = "../../18dof-gpu/data/output/";
    std::string driver_file = inputBasePath + file_name + ".txt";

    // Check if the input file exists
    if (!fs::exists(driver_file)) {
        std::cerr << "Error: Driver input file does not exist: " << driver_file << std::endl;
        return 1;
    }

    // Ensure output directory exists
    if (!fs::exists(outputBasePath)) {
        fs::create_directories(outputBasePath);
    }

    // Vehicle specification
    std::string vehParamsJSON = "../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles, driver_file);

    // Set the threads per block
    solver.SetThreadsPerBlock(threads_per_block);

    // Set the time step of the solver
    solver.SetTimeStep(1e-3);

    // Initialize the states for all vehicles
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);

    // Set output and end time
    solver.SetOutput(outputBasePath + file_name + "_hmmwv18", 100, false, 50);
    solver.SetEndTime(10.0);

    // Solve
    auto start = std::chrono::high_resolution_clock::now();
    solver.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Solve time: " << elapsed.count() << " s\n";
}
