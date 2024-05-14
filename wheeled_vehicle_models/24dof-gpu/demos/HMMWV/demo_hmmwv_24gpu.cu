// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user provided number of HMMWVs (specified with JSON files),
// all operating on the same driver inputs on the GPU. Since the Half Implicit solver is the
// only one supported for the GPU models, that is what is used here. The structure of the API
// is very similar to the CPU version except for the additional requirements of specifying the
// number of vehicles and threads per block.
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

#include "dof24_halfImplicit_gpu.cuh"

namespace fs = std::filesystem;
using namespace d24GPU;

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <total_number_of_vehicles> <threads_per_block>" << std::endl;
        return 1;
    }

    unsigned int num_vehicles = std::stoul(argv[1]);
    unsigned int threads_per_block = std::stoul(argv[2]);

    std::string file_name = "acc3";
    std::string inputPath = "../../24dof-gpu/data/input/" + file_name + ".txt";
    std::string outputPath = "../../24dof-gpu/data/output/";

    // Ensure the input file exists
    if (!fs::exists(inputPath)) {
        std::cerr << "Error: Input file does not exist: " << inputPath << std::endl;
        return 1;
    }

    // Ensure the output directory exists
    if (!fs::exists(outputPath)) {
        fs::create_directories(outputPath);
    }

    std::string vehParamsJSON = "../../24dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../24dof-gpu/data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = "../../24dof-gpu/data/json/HMMWV/suspension.json";

    // Construct the solver
    d24SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, num_vehicles, inputPath);

    solver.SetThreadsPerBlock(threads_per_block);
    solver.SetTimeStep(1e-3);

    // Initialize the states for all vehicles
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    SuspensionState suslf_st;
    SuspensionState susrf_st;
    SuspensionState suslr_st;
    SuspensionState susrr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                      num_vehicles);

    // Output settings, example with partial output to save storage space
    solver.SetOutput(outputPath + file_name + "_hmmwv24", 100, false, 50);

    // Set simulation end time
    solver.SetEndTime(10.0);

    // Solve and measure performance
    auto start = std::chrono::high_resolution_clock::now();
    solver.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Solve time: " << elapsed.count() << " s\n";

    return 0;
}
