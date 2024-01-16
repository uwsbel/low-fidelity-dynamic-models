#include <numeric>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <iostream>
#include <vector>
#include <gtest/gtest.h>

#include "dof18_halfImplicit.h"
#include "dof18_sundials.h"

TEST(dof18Hi, acc_test) {
    // Driver inputs
    std::string driver_file = "../../18dof/data/input/acc3.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../18dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set solver time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    d18::VehicleState veh_st;
    d18::TMeasyState tirelf_st;
    d18::TMeasyState tirerf_st;
    d18::TMeasyState tirelr_st;
    d18::TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Solve
    solver.Solve();

    double expected_final_position = 88.5335;
    double obtained_final_position = solver.m_veh_state._x;
    EXPECT_NEAR(expected_final_position, obtained_final_position, 1);
}

TEST(dof18Sun, outputCheck) {
    // Driver inputs
    std::string driver_file = "../../18dof/data/input/acc3.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../18dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set optional inputs
    // Solver tolerances and maximum step size
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetMaxStep(1e-2);

    // Initialize solver (set initial conditions)
    d18::VehicleState veh_st;
    d18::TMeasyState tirelf_st;
    d18::TMeasyState tirerf_st;
    d18::TMeasyState tirelr_st;
    d18::TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Enable output
    solver.SetOutput("acc3_hmmwv18Sundials.csv");
    solver.SetVerbose(false);

    // Solve without quadratures (no reference path required)
    solver.Solve(false);

    // File path
    std::string filePath = "acc3_hmmwv18Sundials.csv";

    // Create an ifstream object to read from the file
    std::ifstream file(filePath);

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return;
    }

    std::string line;
    std::string lastSecondColumnValue;
    while (getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::getline(ss, token, ' ');        // Skip the first column
        if (std::getline(ss, token, ' ')) {  // Get the second column
            lastSecondColumnValue = token;
        }
    }

    file.close();

    // Convert the second column of the last row to a double
    double value = 0.0;
    try {
        value = std::stod(lastSecondColumnValue);
    } catch (const std::exception& e) {
        std::cerr << "Error converting string to double: " << e.what() << std::endl;
        return;
    }

    double expected_final_position = 90.2196;
    EXPECT_NEAR(expected_final_position, value, 1);
}

TEST(dof18Step, step_acc_test) {
    // Driver inputs
    std::string driver_file = "../../18dof/data/input/acc3.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../18dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    d18::VehicleState veh_st;
    d18::TMeasyState tirelf_st;
    d18::TMeasyState tirerf_st;
    d18::TMeasyState tirelr_st;
    d18::TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    double timeStep = solver.GetStep();
    double endTime = 10;
    double t = 0;
    double new_time = 0;

    // Driver inputs
    double throttle = 0;
    double steering = 0;
    double braking = 0;

    // Start the loop
    while (t < (endTime - timeStep / 10.)) {
        // Inputs based on time provided externally
        if (t > 0.5) {
            throttle = 0.5;
            steering = 0.1;
            braking = 0;
        } else {
            throttle = 0;
            steering = 0;
            braking = 0;
        }
        new_time = solver.IntegrateStep(t, throttle, steering, braking);
        t = new_time;
    }

    double expected_final_position = 62.7973;
    double obtained_final_position = solver.m_veh_state._x;
    EXPECT_NEAR(expected_final_position, obtained_final_position, 1);
}

TEST(dof18StepWithJac, step_with_jacobian_acc_test) {
    // Driver inputs
    std::string driver_file = "../../18dof/data/input/acc3.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../18dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    d18::VehicleState veh_st;
    d18::TMeasyState tirelf_st;
    d18::TMeasyState tirerf_st;
    d18::TMeasyState tirelr_st;
    d18::TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    double timeStep = solver.GetStep();
    double endTime = 10;
    double t = 0;
    double new_time = 0;

    // Driver inputs
    double throttle = 0;
    double steering = 0;
    double braking = 0;

    // Start the loop
    while (t < (endTime - timeStep / 10.)) {
        // Inputs based on time provided externally
        if (t > 0.5) {
            throttle = 0.5;
            steering = 0.1;
            braking = 0;
        } else {
            throttle = 0;
            steering = 0;
            braking = 0;
        }
        new_time = solver.IntegrateStepWithJacobian(t, throttle, steering, braking, true);
        t = new_time;
    }

    double expected_final_position = 62.7973;
    double obtained_final_position = solver.m_veh_state._x;
    EXPECT_NEAR(expected_final_position, obtained_final_position, 1);
}

TEST(dof18HiNr, acc_without_relaxation) {
    // Driver inputs
    std::string driver_file = "../../18dof/data/input/acc05.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../18dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = (char*)"../../18dof/data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    // Since the TMeasyNr tire is used, the TireType must be set
    TireType type = TireType::TMeasyNr;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file, type);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    d18::VehicleState veh_st;
    d18::TMeasyNrState tirelf_st;
    d18::TMeasyNrState tirerf_st;
    d18::TMeasyNrState tirelr_st;
    d18::TMeasyNrState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Solve
    solver.Solve();

    double expected_final_position = 114.973;
    double obtained_final_position = solver.m_veh_state._x;
    EXPECT_NEAR(expected_final_position, obtained_final_position, 1);
}
