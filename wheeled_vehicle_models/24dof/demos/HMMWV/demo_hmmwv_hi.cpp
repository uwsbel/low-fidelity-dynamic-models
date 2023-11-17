#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof24_halfImplicit.h"

using namespace d24;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = (char*)"../data/json/HMMWV/suspension.json";

    // Construct the solver
    d24SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, driver_file);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    SuspensionState suslf_st;
    SuspensionState susrf_st;
    SuspensionState suslr_st;
    SuspensionState susrr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st);

    // Enable output
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_hmmwv24Hi.csv", 100);
    // Solve without quadratures (no reference path required)
    solver.Solve();
}