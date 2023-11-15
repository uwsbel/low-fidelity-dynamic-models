// This demo shows how to use the TMeasy tire without any relaxation
#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_halfImplicit.h"

using namespace d11;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";
    // std::string driver_file = "../data/input/sedan_specific/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    d11SolverHalfImplicit solver;
    TireType type = TireType::TMeasyNr;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file, type);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyNrState tiref_st;
    TMeasyNrState tirer_st;
    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_sedanNr11Hi.csv", 100.);

    // Solve without quadratures (no reference path required)
    solver.Solve();
}