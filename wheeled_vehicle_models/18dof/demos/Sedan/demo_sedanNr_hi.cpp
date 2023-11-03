// This demo shows how to use the TMeasy tire without any relaxation
#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof18_halfImplicit.h"

using namespace d18;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    TireType type = TireType::TMeasyNr;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file, type);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyNrState tirelf_st;
    TMeasyNrState tirerf_st;
    TMeasyNrState tirelr_st;
    TMeasyNrState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Enable output
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_sedanNr18Hi.csv", 100.);

    // Solve without quadratures (no reference path required)
    solver.Solve();
}