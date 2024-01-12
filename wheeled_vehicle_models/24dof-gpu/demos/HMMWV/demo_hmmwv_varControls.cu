// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user 1000 HMMWVs (specified with JSON files), 500 operating on one driver inpput file 
// and the rest operating on another driver input file. Since the Half Implicit solver is the only one supported 
// for the GPU models, that is what is used here.
// Use ./executable_name <threads_per_block>
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

#include "dof24_halfImplicit_gpu.cuh"


using namespace d24GPU;
int main(int argc, char** argv) {
    // Set the total number of vehicles
    unsigned int num_vehicles = 1000;
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[1]);

    // Get two driver files
    std::string file_name_1 = "acc3";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file_1 = "../data/input/" + file_name_1 + ".txt";

    std::string file_name_2 = "double_lane4";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file_2 = "../data/input/" + file_name_2 + ".txt";

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = (char*)"../data/json/HMMWV/suspension.json";

    // Construct the solver
    d24SolverHalfImplicitGPU solver(num_vehicles);

    // First construct half the vehicles for driver file 1
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, 500, driver_file_1);
    // Then construct the other half for driver file 2
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, 500, driver_file_2);

    // Set the threads per block
    solver.SetThreadsPerBlock(threads_per_block);

    // Set the time step of the solver
    solver.SetTimeStep(1e-3);

    // Now we initialize the states -> These are all set to 0 (struct initializer)
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    SuspensionState suslf_st;
    SuspensionState susrf_st;
    SuspensionState suslr_st;
    SuspensionState susrr_st;
    // Again we initialize the same states for all vehicles
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                      num_vehicles);
    // This is the end time of the longer driver file
    // Note: This means that the last control input is applied till the end of the simulation for the shorter driver
    // file
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

    // Extract terminal state of choosen vehicles and print the position
    SimState sim_state_1 = solver.GetSimState(499);
    SimState sim_state_2 = solver.GetSimState(999);

    std::cout << "X Position of vehicle 499: " << sim_state_1._v_states._x << std::endl;
    std::cout << "X Position of vehicle 999: " << sim_state_2._v_states._x << std::endl;
}
