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

#include "dof24_halfImplicit_gpu.cuh"



using namespace d24GPU;
int main(int argc, char** argv) {
    // Get total number of vehicles from command line
    unsigned int num_vehicles = std::stoul(argv[1]);
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[2]);

    std::string file_name = "acc3";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file = "../../24dof-gpu/data/input/" + file_name + ".txt";

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = (char*)"../../24dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../24dof-gpu/data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = (char*)"../../24dof-gpu/data/json/HMMWV/suspension.json";

    // Construct the solver
    d24SolverHalfImplicitGPU solver(num_vehicles);
    // The number of vehicles here sets these parameters and inputs for all these vehicles
    // If there is a need to set different parameters for different vehicles, then the solver
    // needs to be constructed for each vehicle separately (using the same sovler object)
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, num_vehicles, driver_file);

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

    // Enable output for all the vehicles
    // solver.SetOutput("../../24dof-gpu/data/output/" + file_name + "_hmmwv24", 100, true);
    // Enable output for 50 of the vehicles
    solver.SetOutput("../../24dof-gpu/data/output/" + file_name + "_hmmwv24", 100, false, 50);
    // Set the simulation end time -> This is a input that *must* be set by the user
    solver.SetEndTime(10.0);

    // Solve
    // Time the solve
    auto start = std::chrono::high_resolution_clock::now();
    solver.Solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Solve time: " << elapsed.count() << " s\n";
}
