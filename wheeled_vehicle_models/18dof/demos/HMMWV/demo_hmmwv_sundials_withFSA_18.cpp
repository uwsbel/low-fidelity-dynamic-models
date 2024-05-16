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
// In this demo, we additionally demonstrate the use of the Forward Sensitivity
// Analysis (FSA) module. The user thus provides the parameters for which sensitivities
// are desired as well as parameter scales. In case quadratures are desired, the user
// must also provide a reference path. The equations are then solved and the data at a frequency of 10 Hz
// is written to the specified output file.
//
// =============================================================================
#include <iostream>
#include <filesystem>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <vector>

#include "dof18_sundials.h"

namespace fs = std::filesystem;
using namespace d18;

int main(int argc, char** argv) {
    if (argc != 1) {
        std::cerr << "Usage: " << argv[0] << std::endl;
        return 1;
    }

    // Check and ensure output directory exists
    std::string outputBasePath = "../../18dof/data/output/";
    if (!fs::exists(outputBasePath)) {
        fs::create_directories(outputBasePath);
    }

    // Vehicle specification and driver input files
    std::string driver_file = "../../18dof/data/input/acc.txt";
    std::string path_file = "../../18dof/data/reference/chrono_acc.txt";
    std::string vehParamsJSON = "../../18dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = "../../18dof/data/json/HMMWV/tmeasy.json";

    // Ensure input files exist
    if (!fs::exists(driver_file) || !fs::exists(path_file)) {
        std::cerr << "Error: Required input files are missing." << std::endl;
        return 1;
    }

    // Engine torque map parameters
    std::vector<double> params = {258.0,  328.52, 421.4,  497.94, 559.0,  607.16, 641.56, 665.64, 678.54, 681.98,
                                  677.68, 665.64, 648.44, 626.08, 599.42, 571.04, 540.08, 509.98, 479.88, -344.0};
    std::vector<double> scales = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                  100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

    // Construct the solver
    d18SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set solver options
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetReferencePath(path_file);
    solver.SetSensitivityParameters(ParamFlag::ENGINE_MAP, params);
    solver.SetSensitivityParameterScales(scales);

    // Initialize the solver
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Output settings and solve
    solver.SetOutput(outputBasePath + "output.txt");
    solver.SetVerbose(false);
    auto cf = solver.Evaluate_FSA(true);

    std::cout << "---------------------------------" << std::endl;
    std::cout << "Return flag:   " << cf.success << std::endl;
    std::cout << "Cost function: " << cf.value << std::endl;
    std::cout << "Cost gradient: " << std::endl;
    for (const auto& g : cf.gradient)
        std::cout << "  " << g << std::endl;

    return 0;
}
