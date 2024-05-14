// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A HMMWV vehicle is defined using example JSON files.
// In addition, the user is required to provide a driver input file. 
// Example driver input files are provided in the data/input folder.
// The Sundials solver is then Constructed and Initialized.
// Since the sundials solver is used, the user can optionally
// set the solver tolerances and the maximum solver time-step.
// The equations are then solved and the data at a frequency of 10 Hz
// is written to the specified output file.
//
// =============================================================================
#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_sundials.h"

using namespace d11;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../../11dof/data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../11dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../11dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set optional inputs
    // Solver tolerances and maximum step size
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetMaxStep(1e-2);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput("../../11dof/data/output/" + std::string(argv[1]) + "_hmmwv11Sundials.csv");
    solver.SetVerbose(false);

    // Solve without quadratures (no reference path required)
    solver.Solve(false);
}
