// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user provided number of HMMWVs (specified with JSON files) on a step-by-step
// basis. The "step" at which a new control input is provided can be set with SetKernelSimTime as shown here.
// Since the Half Implicit solver is the only one supported for the GPU models, that is what is used here.
// When the solver is used in a step-by-step manner, the output is not stored in a file (unlike the CPU models).
// However, access to the vehicle states every control time step is provided through the GetSimState function.
// Use ./executable_name <total_number_of_vehicles> <threads_per_block>
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

#include "dof18_halfImplicit_gpu.cuh"

namespace fs = std::filesystem;
using namespace d18GPU;

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <total_number_of_vehicles> <threads_per_block>" << std::endl;
        return 1;
    }

    unsigned int num_vehicles = std::stoul(argv[1]);
    unsigned int threads_per_block = std::stoul(argv[2]);
    std::string file_name = "acc3";
    std::string inputPath = "../../18dof-gpu/data/input/" + file_name + ".txt";

    // Ensure the input file exists
    if (!fs::exists(inputPath)) {
        std::cerr << "Error: Input file does not exist: " << inputPath << std::endl;
        return 1;
    }

    std::string vehParamsJSON = "../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicitGPU solver(num_vehicles);
    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles);

    solver.SetThreadsPerBlock(threads_per_block);
    solver.SetTimeStep(1e-3);
    solver.SetKernelSimTime(0.1);

    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);

    double endTime = 10.0;
    double timeStep = solver.GetStep();
    double t = 0;
    double new_time = 0;
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);

    double throttle;
    double steering;
    double braking;
    while (t < (endTime - timeStep / 10.)) {
        if (t > 1) {
            throttle = 0.5;
            steering = 0.0;
            braking = 0.0;
        } else {
            throttle = 0.0;
            steering = 0.0;
            braking = 0.0;
        }

        new_time = solver.SolveStep(t, steering, throttle, braking);  // Solve for the current time
        t = new_time;
    }

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Solve time (ms): " << milliseconds << "\n";

    SimState sim_state_1 = solver.GetSimState(0);
    std::cout << "X Position of vehicle 1: " << sim_state_1._veh_state._x << std::endl;
}
