#include <cuda.h>
#include <iostream>
#include <random>
#include <cuda_runtime.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <gtest/gtest.h>

#include "dof11_halfImplicit_gpu.cuh"

TEST(dof11_gpu, acc_test) {
    unsigned int num_vehicles = 50;
    unsigned int threads_per_block = 32;
    std::string driver_file = "../../11dof-gpu/data/input/acc3.txt";

    std::string vehParamsJSON = (char*)"../../11dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../11dof-gpu/data/json/HMMWV/tmeasy.json";

    d11SolverHalfImplicitGPU solver(num_vehicles);

    solver.Construct(vehParamsJSON, tireParamsJSON, num_vehicles, driver_file);

    solver.SetThreadsPerBlock(threads_per_block);

    solver.SetTimeStep(1e-3);

    d11GPU::VehicleState veh_st;
    d11GPU::TMeasyState tiref_st;
    d11GPU::TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st, num_vehicles);

    solver.SetEndTime(10.0);

    solver.Solve();
    double expected_final_position = 70.6758;
    d11GPU::SimState sim_state_1 = solver.GetSimState(0);
    double obtained_final_position = sim_state_1._veh_state._x;
    EXPECT_NEAR(expected_final_position, obtained_final_position, 1);
}

TEST(dof11_gpu, variable_controls) {
    unsigned int num_vehicles = 1000;
    unsigned int threads_per_block = 32;

    std::string file_name_1 = "acc3";
    std::string driver_file_1 = "../../11dof-gpu/data/input/" + file_name_1 + ".txt";

    std::string file_name_2 = "double_lane4";
    std::string driver_file_2 = "../../11dof-gpu/data/input/" + file_name_2 + ".txt";

    std::string vehParamsJSON = (char*)"../../11dof-gpu/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../11dof-gpu/data/json/HMMWV/tmeasy.json";

    d11SolverHalfImplicitGPU solver(num_vehicles);

    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_1);
    solver.Construct(vehParamsJSON, tireParamsJSON, 500, driver_file_2);

    solver.SetThreadsPerBlock(threads_per_block);

    solver.SetTimeStep(1e-3);

    d11GPU::VehicleState veh_st;
    d11GPU::TMeasyState tiref_st;
    d11GPU::TMeasyState tirer_st;
    solver.Initialize(veh_st, tiref_st, tirer_st, num_vehicles);
    solver.SetEndTime(22.0);

    solver.Solve();

    d11GPU::SimState sim_state_1 = solver.GetSimState(499);
    d11GPU::SimState sim_state_2 = solver.GetSimState(999);
    double should_be1 = 198.72;
    double should_be2 = 104.805;
    double check1 = sim_state_1._veh_state._x;
    double check2 = sim_state_2._veh_state._x;
    EXPECT_NEAR(check1, should_be1, 1);
    EXPECT_NEAR(check2, should_be2, 1);
}