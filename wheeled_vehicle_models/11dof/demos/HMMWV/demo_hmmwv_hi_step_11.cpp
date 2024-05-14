// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A HMMWV vehicle is defined using example JSON files.
// In this case, a driver input file is not required as the vehicle can also be simulated step-by-step.
// The Half-Implicit solver is then Constructed, Initialized and solved. Data at the specified
// output frequency is written to the specified output file.
//
// =============================================================================
#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_halfImplicit.h"

using namespace d11;

int main(int argc, char** argv) {
    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../11dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../11dof/data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d11SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tiref_st;
    TMeasyState tirer_st;

    solver.Initialize(veh_st, tiref_st, tirer_st);

    // Enable output
    solver.SetOutput("../../11dof/data/output/" + std::string(argv[1]) + "_hmmwv11HiStep.csv", 100);

    double timeStep = solver.GetStep();
    double endTime = 10;
    double t = 0;
    double new_time = 0;

    // Driver inputs
    double throttle = 0;
    double steering = 0;
    double braking = 0;

    // Start the loop
    while (t < (endTime - timeStep / 10.)) {
        // Inputs based on time provided externally
        if(t > 0.5) {
            throttle = 0.5;
            steering = 0.1;
            braking = 0;
        }
        else {
            throttle = 0;
            steering = 0;
            braking = 0;
        }
        new_time = solver.IntegrateStep(t, throttle, steering, braking);
        t = new_time;
    }
    // Write the writer to file (required when using the step-by-step solver)
    solver.WriteToFile();
}