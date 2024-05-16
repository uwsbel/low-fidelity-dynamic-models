// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating 1000 HMMWVs (specified with JSON files), split between two driver input files.
// The Half Implicit solver, which is designed for GPU models, is utilized here.
// Usage: ./executable_name <threads_per_block>
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
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <threads_per_block>" << std::endl;
        return 1;
    }

    unsigned int num_vehicles = 1000;
    unsigned int threads_per_block = std::stoul(argv[1]);

    std::string file_name_1 = "acc3";
    std::string inputPath1 = "../../24dof-gpu/data/input/" + file_name_1 + ".txt";

    std::string file_name_2 = "double_lane4";
    std::string inputPath2 = "../../24dof-gpu/data/input/" + file_name_2 + ".txt";

    if (!fs::exists(inputPath1) || !fs::exists(inputPath2)) {
        std::cerr << "Error: One or both driver input files do not exist." << std::endl;
        return 1;
    }

    std::string vehParamsJSON = "../../24dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../24dof-gpu/data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = "../../24dof-gpu/data/json/HMMWV/suspension.json";

    d24SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, 500, inputPath1);
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, 500, inputPath2);

    solver.SetThreadsPerBlock(threads_per_block);
    solver.SetTimeStep(1e-3);

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

    std::string outputPath = "../../24dof-gpu/data/output/";
    if (!fs::exists(outputPath)) {
        fs::create_directories(outputPath);
    }

    solver.SetEndTime(22.0);

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

    SimState sim_state_1 = solver.GetSimState(499);
    SimState sim_state_2 = solver.GetSimState(999);

    std::cout << "X Position of vehicle 499: " << sim_state_1._v_states._x << std::endl;
    std::cout << "X Position of vehicle 999: " << sim_state_2._v_states._x << std::endl;

    return 0;
}
