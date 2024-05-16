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

#include <iostream>
#include <filesystem>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <stdexcept>

#include "dof11_halfImplicit.h"

namespace fs = std::filesystem;
using namespace d11;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <driverInputFile>" << std::endl;
        return 1;
    }

    // Construct file paths
    std::string baseInputPath = "../../11dof/data/input/";
    std::string baseOutputPath = "../../11dof/data/output/";
    std::string inputFileName = baseInputPath + argv[1] + ".txt";
    std::string outputFileName = baseOutputPath + argv[1] + "_hmmwv11Hi.csv";

    // Check if input file exists
    if (!fs::exists(inputFileName)) {
        std::cerr << "Error: Input file does not exist: " << inputFileName << std::endl;
        return 1;
    }

    // Ensure output directory exists
    if (!fs::exists(baseOutputPath)) {
        fs::create_directories(baseOutputPath);
    }

    // Vehicle specification
    std::string vehParamsJSON = "../../11dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../11dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, inputFileName);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput(outputFileName, 100);
    // Solve without quadratures (no reference path required)
    solver.Solve();

    return 0;
}
