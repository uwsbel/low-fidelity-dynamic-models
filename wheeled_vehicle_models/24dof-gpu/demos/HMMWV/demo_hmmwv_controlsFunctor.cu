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

// A simple functor for demonstration
struct MyFunctor {
    // It is imperitve that these input outputs need to be maintained
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

// Use ./executable_name <total_number_of_vehicles> <threads_per_block>

using namespace d24GPU;
int main(int argc, char** argv) {
    // Get total number of vehicles from command line
    unsigned int num_vehicles = std::stoul(argv[1]);
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[2]);
    std::string file_name = "acc3";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file = "../data/input/" + file_name + ".txt";

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = (char*)"../data/json/HMMWV/suspension.json";

    // Construct the solver
    d24SolverHalfImplicitGPU solver(num_vehicles);
    // The number of vehicles here sets these parameters and inputs for all these vehicles
    // If there is a need to set different parameters for different vehicles, then the solver
    // needs to be constructed for each vehicle separately (using the same sovler object)
    // No driver file
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, num_vehicles, driver_file);

    // Set the threads per block
    solver.SetThreadsPerBlock(threads_per_block);

    // Set the time step of the solver
    solver.SetTimeStep(1e-3);

    // Decide on the "step" timestep and set it here
    double control_time_step = 1e-1;
    solver.SetKernelSimTime(control_time_step);

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

    // NOTE: SolveStep does not support storing output so both of these need to stay commented
    // solver.SetOutput("../data/output/" + file_name + "_hmmwv24", 100, true);
    // Enable output for 50 of the vehicles
    // solver.SetOutput("../data/output/" + file_name + "_hmmwv24step", 100, false, 50);

    // Initialize the controls functor
    MyFunctor myControlsFunctor;
    double endTime = 10.0;
    double timeStep = solver.GetStep();
    double t = 0;
    double new_time = 0;
    // Now solve in loop
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
    // Extract terminal state of choosen vehicles and print the position
    SimState sim_state_1 = solver.GetSimState(0);

    std::cout << "X Position of vehicle 1: " << sim_state_1._v_states._x << std::endl;
}
