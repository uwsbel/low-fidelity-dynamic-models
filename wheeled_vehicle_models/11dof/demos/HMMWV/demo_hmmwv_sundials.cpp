#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_sundials.h"

using namespace d11;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set optional inputs
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetMaxStep(1e-2);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_hmmwv18Sundials.csv");
    solver.SetVerbose(false);

    // Solve without quadratures (no reference path required)
    solver.Solve(false);
}
