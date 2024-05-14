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
#include <iostream>
#include <filesystem>
#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_sundials.h"

namespace fs = std::filesystem;
using namespace d11;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <inputFileName>" << std::endl;
        return 1;
    }

    // Construct file paths
    std::string inputBasePath = "../../11dof/data/input/";
    std::string outputBasePath = "../../11dof/data/output/";
    std::string inputFileName = inputBasePath + argv[1] + ".txt";
    std::string outputFileName = outputBasePath + argv[1] + "_hmmwv11Sundials.csv";

    // Check if input file exists
    if (!fs::exists(inputFileName)) {
        std::cerr << "Error: Input file does not exist: " << inputFileName << std::endl;
        return 1;
    }

    // Ensure output directory exists
    if (!fs::exists(outputBasePath)) {
        fs::create_directories(outputBasePath);
    }

    // Vehicle specification
    std::string vehParamsJSON = "../../11dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../11dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, inputFileName);

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
    solver.SetOutput(outputFileName);
    solver.SetVerbose(false);

    // Solve without quadratures (no reference path required)
    solver.Solve(false);

    return 0;
}
