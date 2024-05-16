// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A Sedan vehicle is defined using example JSON files.
// In addition, the user is required to provide a driver input file.
// Example driver input files are provided in the data/input folder.
// The Half-Implicit solver is then Constructed, Initialized and solved.
// In this demo the TMeasy tire without relaxation is used (TMeasyNr). This
// requires setting the TireType as shown. Data at the specified
// output frequency is written to the specified output file.
//
// =============================================================================
#include <iostream>
#include <filesystem>
#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof24_halfImplicit.h"

namespace fs = std::filesystem;
using namespace d24;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <inputFileName>" << std::endl;
        return 1;
    }

    // Setup file paths
    std::string baseInputPath = "../../24dof/data/input/";
    std::string baseOutputPath = "../../24dof/data/output/";
    std::string inputFileName = baseInputPath + argv[1] + ".txt";
    std::string outputFileName = baseOutputPath + argv[1] + "_sedanNr24Hi.csv";

    // Check if the input file exists
    if (!fs::exists(inputFileName)) {
        std::cerr << "Error: Input file does not exist: " << inputFileName << std::endl;
        return 1;
    }

    // Ensure the output directory exists
    if (!fs::exists(baseOutputPath)) {
        fs::create_directories(baseOutputPath);
    }

    // Vehicle specification files
    std::string vehParamsJSON = "../../24dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = "../../24dof/data/json/Sedan/tmeasyNr.json";
    std::string susParamsJSON = "../../24dof/data/json/Sedan/suspension.json";

    // Construct the solver with specified tire type
    TireType type = TireType::TMeasyNr;
    d24SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, inputFileName, type);

    // Set simulation parameters
    solver.SetTimeStep(1e-3);

    // Initialize solver with initial conditions
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

    // Enable output and solve
    solver.SetOutput(outputFileName, 100);
    solver.Solve();

    return 0;
}
