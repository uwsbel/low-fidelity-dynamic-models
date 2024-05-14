// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A Sedan vehicle is defined using example JSON files.
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

#include "dof24_sundials.h"

namespace fs = std::filesystem;
using namespace d24;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <inputFileNameWithoutExtension>" << std::endl;
        return 1;
    }

    std::string baseInputPath = "../../24dof/data/input/";
    std::string baseOutputPath = "../../24dof/data/output/";
    std::string inputFileName = baseInputPath + argv[1] + ".txt";
    std::string outputFileName = baseOutputPath + argv[1] + "_sedanNr24Sundials.csv";

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
    std::string vehParamsJSON = "../../24dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON = "../../24dof/data/json/Sedan/tmeasyNr.json";
    std::string susParamsJSON = "../../24dof/data/json/Sedan/suspension.json";

    // Construct the solver with specified tire type
    TireType type = TireType::TMeasyNr;
    d24SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON, inputFileName, type);

    // Set solver options
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetMaxStep(1e-2);

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

    // Enable output
    solver.SetOutput(outputFileName);

    // Solve without quadratures (no reference path required)
    solver.Solve(false);

    return 0;
}
