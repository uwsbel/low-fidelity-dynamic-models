// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating 1000 HMMWVs (specified with JSON files), with 500 operating on one driver input file
// and the rest on another driver input file. Since the Half Implicit solver is the only one supported
// for the GPU models, that is what is used here.
// Use ./executable_name <threads_per_block>
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
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <threads_per_block>" << std::endl;
        return 1;
    }

    unsigned int num_vehicles = 1000;                      // Total number of vehicles is predefined as 1000
    unsigned int threads_per_block = std::stoul(argv[1]);  // Number of threads per block from command line

    std::string file_name_1 = "acc3";
    std::string driver_file_1 = "../../18dof-gpu/data/input/" + file_name_1 + ".txt";
    std::string file_name_2 = "double_lane4";
    std::string driver_file_2 = "../../18dof-gpu/data/input/" + file_name_2 + ".txt";

    // Check if the input files exist
    if (!fs::exists(driver_file_1) || !fs::exists(driver_file_2)) {
        std::cerr << "Error: One or both driver input files do not exist." << std::endl;
        return 1;
    }

    std::string vehParamsJSON = "../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver for 1000 vehicles
    d18SolverHalfImplicitGPU solver(num_vehicles);

    // Construct half the vehicles with driver file 1 and the other half with driver file 2
    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_1);
    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_2);

    solver.SetThreadsPerBlock(threads_per_block);
    solver.SetTimeStep(1e-3);

    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;

    // Initialize the states for all vehicles
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);
    solver.SetEndTime(22.0);  // Setting the end time based on the longer driver file

    // Timing the simulation
    auto start = std::chrono::high_resolution_clock::now();
    solver.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Solve time: " << elapsed.count() << " s\n";

    // Extract and display terminal state of chosen vehicles
    SimState sim_state_1 = solver.GetSimState(499);
    SimState sim_state_2 = solver.GetSimState(999);

    std::cout << "X Position of vehicle 499: " << sim_state_1._veh_state._x << std::endl;
    std::cout << "X Position of vehicle 999: " << sim_state_2._veh_state._x << std::endl;
}
