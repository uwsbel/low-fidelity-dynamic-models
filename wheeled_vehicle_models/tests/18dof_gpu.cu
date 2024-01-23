#include <cuda.h>
#include <iostream>
#include <random>
#include <cuda_runtime.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <gtest/gtest.h>

#include "dof18_halfImplicit_gpu.cuh"

TEST(dof18_gpu, acc_test) {
    // Get total number of vehicles from command line
    unsigned int num_vehicles = 50;
    // Set the threads per block from command line
    unsigned int threads_per_block = 32;
    // Driver inputs
    std::string driver_file = "../../18dof-gpu/data/input/acc3.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicitGPU solver(num_vehicles);
    // The number of vehicles here sets these parameters and inputs for all these vehicles
    // If there is a need to set different parameters for different vehicles, then the solver
    // needs to be constructed for each vehicle separately (using the same sovler object)
    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles, driver_file);

    // Set the threads per block
    solver.SetThreadsPerBlock(threads_per_block);

    // Set the time step of the solver
    solver.SetTimeStep(1e-3);

    // Now we initialize the states -> These are all set to 0 (struct initializer)
    d18GPU::VehicleState veh_st;
    d18GPU::TMeasyState tirelf_st;
    d18GPU::TMeasyState tirerf_st;
    d18GPU::TMeasyState tirelr_st;
    d18GPU::TMeasyState tirerr_st;
    // Again we initialize the same states for all vehicles
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);

    // Set the simulation end time -> This is a input that *must* be set by the user
    solver.SetEndTime(10.0);

    // Solve
    solver.Solve();
    double expected_final_position = 88.5335;
    d18GPU::SimState sim_state_1 = solver.GetSimState(0);
    double obtained_final_position = sim_state_1._veh_state._x;
    EXPECT_NEAR(expected_final_position, obtained_final_position, 1);
}

TEST(dof18_gpu, variable_controls) {
    unsigned int num_vehicles = 1000;
    unsigned int threads_per_block = 32;

    std::string file_name_1 = "acc3";
    std::string driver_file_1 = "../../18dof-gpu/data/input/" + file_name_1 + ".txt";

    std::string file_name_2 = "double_lane4";
    std::string driver_file_2 = "../../18dof-gpu/data/input/" + file_name_2 + ".txt";

    std::string vehParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof-gpu/data/json/HMMWV/tmeasy.json";

    d18SolverHalfImplicitGPU solver(num_vehicles);

    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_1);
    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_2);

    solver.SetThreadsPerBlock(threads_per_block);

    solver.SetTimeStep(1e-3);

    d18GPU::VehicleState veh_st;
    d18GPU::TMeasyState tirelf_st;
    d18GPU::TMeasyState tirerf_st;
    d18GPU::TMeasyState tirelr_st;
    d18GPU::TMeasyState tirerr_st;
    // Again we initialize the same states for all vehicles
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, num_vehicles);
    solver.SetEndTime(22.0);

    solver.Solve();

    d18GPU::SimState sim_state_1 = solver.GetSimState(499);
    d18GPU::SimState sim_state_2 = solver.GetSimState(999);
    double should_be1 = 241.112;
    double should_be2 = 106.548;
    double check1 = sim_state_1._veh_state._x;
    double check2 = sim_state_2._veh_state._x;
    EXPECT_NEAR(check1, should_be1, 1);
    EXPECT_NEAR(check2, should_be2, 1);
}
