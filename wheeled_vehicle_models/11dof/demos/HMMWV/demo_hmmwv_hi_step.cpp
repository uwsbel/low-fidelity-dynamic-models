#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_halfImplicit.h"

using namespace d11;

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_hmmwv11HiStep.csv", 100);

    double timeStep = solver.GetStep();
    double endTime = solver.GetEndT();
    double t = 0;
    double new_time = 0;
    DriverData driver_data = solver.GetDriverData();  // Get the driver data associated with input file
    // Start the loop
    while (t < (endTime - timeStep / 10.)) {
        auto controls = GetDriverInput(t, driver_data);  // Get the controls for the current time
        new_time = solver.IntegrateStep(t, controls.m_throttle, controls.m_steering, controls.m_braking);
        t = new_time;
    }
    solver.WriteToFile();
}