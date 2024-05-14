// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A HMMWV vehicle is defined using example JSON files.
// In addition, the user is required to provide a driver input file. 
// Example driver input files are provided in the data/input folder.
// The Half-Implicit solver is then Constructed, Initialized and solved. Data at the specified
// output frequency is written to the specified output file.
//
// =============================================================================

#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_halfImplicit.h"

using namespace d11;

int main(int argc, char** argv) {
    // Driver inputs
    std::string driver_file = "../../11dof/data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../11dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../11dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput("../../11dof/data/output/" + std::string(argv[1]) + "_hmmwv11Hi.csv", 100);
    // Solve without quadratures (no reference path required)
    solver.Solve();
}