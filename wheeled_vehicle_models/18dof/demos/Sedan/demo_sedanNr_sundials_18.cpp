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

#include "dof18_sundials.h"

namespace fs = std::filesystem;
using namespace d18;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <inputSubName>" << std::endl;
        return 1;
    }

    // Construct file paths
    std::string inputBasePath = "../../18dof/data/input/";
    std::string outputBasePath = "../../18dof/data/output/";
    std::string inputFileName = inputBasePath + argv[1] + ".txt";
    std::string outputFileName = outputBasePath + argv[1] + "_sedanNr18Sundials.csv";

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
    std::string vehParamsJSON = "../../18dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = "../../18dof/data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    TireType type = TireType::TMeasyNr;
    d18SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, inputFileName, type);

    // Set solver options
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetMaxStep(1e-2);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyNrState tirelf_st;
    TMeasyNrState tirerf_st;
    TMeasyNrState tirelr_st;
    TMeasyNrState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Enable output
    solver.SetOutput(outputFileName);

    // Solve without quadratures (no reference path required)
    solver.Solve(false);

    return 0;
}
