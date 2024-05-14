// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// This demo describes simulating user provided number of HMMWVs (specified with JSON files) on a step-by-step
// basis. The "step" at which a new control input is provided can be set with SetKernelSimTime as shown here.
// Additionally, shown here is a way to provide a function object (functor) to the solver to provide the
// control inputs. This is done by creating a functor and passing it to the solver's SolveStep function.
// The functor needs to have an operator() function that takes in the current time and a pointer to the
// DriverInput struct. This functionallity is useful when the control Functor represents some sort of controller.
// Since the Half Implicit solver is the only one supported for the GPU models, that is what is used here.
// When the solver is used in a step-by-step manner, the output is not stored in a file (unlike the CPU models).
// However, access to the vehicle states every control time step is provided through the GetSimState function.
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

#include "dof18_halfImplicit_gpu.cuh"

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

using namespace d18GPU;
int main(int argc, char** argv) {
    // Get total number of vehicles from command line
    unsigned int num_vehicles = std::stoul(argv[1]);
    // Set the threads per block from command line
    unsigned int threads_per_block = std::stoul(argv[2]);
    std::string file_name = "acc3";
    // Driver inputs -> All vehicles have the same driver inputs
    std::string driver_file = "../../18dof-gpu/data/input/" + file_name + ".txt";

    // Vehicle specification -> We assume that all vehicles have the same parameters
    std::string vehParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicitGPU solver(num_vehicles);
    // The number of vehicles here sets these parameters and inputs for all these vehicles
    // If there is a need to set different parameters for different vehicles, then the solver
    // needs to be constructed for each vehicle separately (using the same sovler object)
    // No driver file
    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles);

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
    // Again we initialize the same states for all vehicles
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);

    // NOTE: SolveStep does not support storing output so both of these need to stay commented
    // solver.SetOutput("../../18dof-gpu/data/output/" + file_name + "_hmmwv18", 100, true);
    // Enable output for 50 of the vehicles
    // solver.SetOutput("../../18dof-gpu/data/output/" + file_name + "_hmmwv18step", 100, false, 50);

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

    std::cout << "X Position of vehicle 1: " << sim_state_1._veh_state._x << std::endl;
}
