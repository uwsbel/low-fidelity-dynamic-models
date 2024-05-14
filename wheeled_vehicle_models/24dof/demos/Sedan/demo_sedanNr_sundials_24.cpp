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

#include "dof24_sundials.h"

using namespace d24;

int main(int argc, char** argv) {
    // Driver inputs
    std::string driver_file = "../../24dof/data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../24dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = (char*)"../../24dof/data/json/Sedan/tmeasyNr.json";
    std::string susParamsJSON = (char*)"../../24dof/data/json/Sedan/suspension.json";
    // Construct the solver
    d24SolverSundials solver;
    // Since the TMeasyNr tire is used, the TireType must be set
    TireType type = TireType::TMeasyNr;
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, driver_file, type);

    // Set optional inputs
    // Solver tolerances and maximum step size
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetMaxStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyNrState tirelf_st;
    TMeasyNrState tirerf_st;
    TMeasyNrState tirelr_st;
    TMeasyNrState tirerr_st;
    SuspensionState suslf_st;
    SuspensionState susrf_st;
    SuspensionState suslr_st;
    SuspensionState susrr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st);

    // Enable output
    solver.SetOutput("../../24dof/data/output/" + std::string(argv[1]) + "_sedanNr24Sundials.csv");

    // Solve without quadratures (no reference path required)
    // False flag solves without forward sensitivity analysis
    solver.Solve(false);
}