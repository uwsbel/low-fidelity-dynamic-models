// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A HMMWV vehicle is defined using example JSON files.
// In addition, the user is required to provide a driver input file. 
// Example driver input files are provided in the data/input folder.
// The Half-Implicit solver is then Constructed, Initialized and solved.
// In this demo the TMeasy tire without relaxation is used (TMeasyNr). This 
// requires setting the TireType as shown. Data at the specified
// output frequency is written to the specified output file.
//
// =============================================================================

#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof18_halfImplicit.h"

using namespace d18;

int main(int argc, char** argv) {
    // Driver inputs
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasyNr.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    // Since the TMeasyNr tire is used, the TireType must be set
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
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_hmmwvNr18Hi.csv", 100);
    // Solve
    solver.Solve();
}