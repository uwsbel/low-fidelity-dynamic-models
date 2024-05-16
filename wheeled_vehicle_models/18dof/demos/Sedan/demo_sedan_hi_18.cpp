// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A Sedan vehicle is defined using example JSON files.
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

#include "dof18_halfImplicit.h"

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
    std::string outputFileName = outputBasePath + argv[1] + "_sedan18Hi.csv";

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
    std::string tireParamsJSON = "../../18dof/data/json/Sedan/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, inputFileName);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Enable output
    solver.SetOutput(outputFileName, 100);

    // Solve
    solver.Solve();

    return 0;
}
