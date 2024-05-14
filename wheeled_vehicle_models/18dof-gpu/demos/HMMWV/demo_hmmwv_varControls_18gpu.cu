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

#include "dof18_halfImplicit_gpu.cuh"


using namespace d18GPU;
int main(int argc, char** argv) {
    // Set the total number of vehicles
    unsigned int num_vehicles = 1000;
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[1]);

    // Get two driver files
    std::string file_name_1 = "acc3";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file_1 = "../../18dof-gpu/data/input/" + file_name_1 + ".txt";

    std::string file_name_2 = "double_lane4";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file_2 = "../../18dof-gpu/data/input/" + file_name_2 + ".txt";

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicitGPU solver(num_vehicles);

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
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    // Again we initialize the same states for all vehicles
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);
    // This is the end time of the longer driver file
    // Note: This means that the last control input is applied till the end of the simulation for the shorter driver
    // file
    solver.SetEndTime(22.0);

    // Solve
    // Time the solve
    auto start = std::chrono::high_resolution_clock::now();
    solver.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Solve time: " << elapsed.count() << " s\n";

    // Extract terminal state of choosen vehicles and print the position
    SimState sim_state_1 = solver.GetSimState(499);
    SimState sim_state_2 = solver.GetSimState(999);

    std::cout << "X Position of vehicle 499: " << sim_state_1._veh_state._x << std::endl;
    std::cout << "X Position of vehicle 999: " << sim_state_2._veh_state._x << std::endl;
}
