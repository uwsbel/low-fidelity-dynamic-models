#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof18_sundials.h"

using namespace d18;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/acc.txt";
    std::string path_file = "../data/reference/chrono_acc.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";

    // Parameters (engine torque map)
    ////std::vector<double> params = {300, 382, 490, 579, 650, 706, 746, 774, 789, 793,
    ////                              788, 774, 754, 728, 697, 664, 628, 593, 558, -400};

    std::vector<double> params = {258.0,  328.52, 421.4,  497.94, 559.0,  607.16, 641.56, 665.64, 678.54, 681.98,
                                  677.68, 665.64, 648.44, 626.08, 599.42, 571.04, 540.08, 509.98, 479.88, -344.0};

    std::vector<double> scales = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                  100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

    // Construct the solver
    d18SolverSundials solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set optional inputs
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetReferencePath(path_file);
    solver.SetSensitivityParameters(ParamFlag::ENGINE_MAP, params);
    solver.SetSensitivityParameterScales(scales);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Enable output
    solver.SetOutput("output.txt");
    solver.SetVerbose(false);

    // Solve without quadratures (no reference path required)
    ////solver.Solve(false);

    // Solve with quadratures (must provide reference path)
    auto cf = solver.Evaluate_FSA(true);

    std::cout << "---------------------------------" << std::endl;
    std::cout << "Return flag:   " << cf.success << std::endl;
    std::cout << "Cost function: " << cf.value << std::endl;
    std::cout << "Cost gradient: " << std::endl;
    for (const auto& g : cf.gradient)
        std::cout << "  " << g << std::endl;

    // Cost function gradient using finite differences
    std::cout << "=================================" << std::endl;
    std::copy(params.begin(), params.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << std::endl;

    solver.SetOutput("output_base.txt");
    auto cf1 = solver.Evaluate_FSA(false);
    std::cout << "\n Cost Function = " << cf1.value << std::endl;

    double delta = 1e-3;
    std::vector<double> cf_gradient_fd(solver.GetNumParameters());
    for (int is = 0; is < solver.GetNumParameters(); is++) {
        params[is] += delta;

        std::cout << "---------------------------------" << std::endl;
        std::cout << "is = " << is << std::endl;
        std::copy(params.begin(), params.end(), std::ostream_iterator<double>(std::cout, " "));
        std::cout << std::endl;

        solver.SetOutput("output_" + std::to_string(is) + ".txt");
        solver.SetSensitivityParameters(ParamFlag::ENGINE_MAP, params);
        auto cf2 = solver.Evaluate_FSA(false);
        std::cout << "\n Cost Function = " << cf2.value << std::endl;

        cf_gradient_fd[is] = (cf2.value - cf1.value) / delta;

        params[is] -= delta;
    }
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Cost gradient: " << std::endl;
    for (int is = 0; is < solver.GetNumParameters(); is++) {
        std::cout << "  " << cf.gradient[is] << "\t" << cf_gradient_fd[is] << std::endl;
    }
}
