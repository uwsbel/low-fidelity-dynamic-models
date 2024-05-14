// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user-provided number of HMMWVs (specified with JSON files) on a step-by-step
// basis using the Half Implicit solver on GPU. It includes functionality for setting control inputs through a functor.
// The API setup requires specifying the number of vehicles and threads per block.
// Usage: ./executable_name <total_number_of_vehicles> <threads_per_block>
//
// =============================================================================
#include <cuda.h>
#include <iostream>
#include <filesystem>
#include <cuda_runtime.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "dof24_halfImplicit_gpu.cuh"

namespace fs = std::filesystem;
using namespace d24GPU;

// Functor for control inputs
struct MyFunctor {
    __device__ void operator()(double t, DriverInput* controls) {
        if (t < 1) {
            controls->m_steering = 0.0;
            controls->m_throttle = 0.0;
            controls->m_braking = 0.0;
        } else {
            controls->m_steering = 0.0;
            controls->m_throttle = 0.5;
            controls->m_braking = 0.0;
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <total_number_of_vehicles> <threads_per_block>" << std::endl;
        return 1;
    }

    unsigned int num_vehicles = std::stoul(argv[1]);
    unsigned int threads_per_block = std::stoul(argv[2]);

    std::string file_name = "acc3";
    std::string inputPath = "../../24dof-gpu/data/input/" + file_name + ".txt";

    // Verify the existence of the input file
    if (!fs::exists(inputPath)) {
        std::cerr << "Error: Input file does not exist: " << inputPath << std::endl;
        return 1;
    }

    std::string vehParamsJSON = "../../24dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../24dof-gpu/data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = "../../24dof-gpu/data/json/HMMWV/suspension.json";

    // Construct the solver with GPU-specific settings
    d24SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, num_vehicles, inputPath);

    solver.SetThreadsPerBlock(threads_per_block);
    solver.SetTimeStep(1e-3);
    solver.SetKernelSimTime(0.1);  // Set control input timestep

    // Initialize vehicle states
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

    // Use functor for control inputs
    MyFunctor myControlsFunctor;
    double endTime = 10.0;
    double timeStep = solver.GetStep();
    double t = 0;
    double new_time = 0;

    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);

    while (t < (endTime - timeStep / 10.)) {
        new_time = solver.SolveStep(t, myControlsFunctor);  // Solve for the current time
        t = new_time;
    }

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Solve time (ms): " << milliseconds << "\n";

    // Extract and print the position of a vehicle
    SimState sim_state_1 = solver.GetSimState(0);
    std::cout << "X Position of vehicle 1: " << sim_state_1._v_states._x << std::endl;

    return 0;
}
