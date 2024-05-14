// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// A HMMWV vehicle is defined using example JSON files.
// In this case, a driver input file is not required as the vehicle can also be simulated step-by-step.
// In addition, this demo shows how to use the Jacobian functionality of the Half-Implicit solver can be used.
// The Half-Implicit solver is then Constructed, Initialized and solved. Data at the specified
// output frequency is written to the specified output file.
//
// =============================================================================
#include <numeric>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "dof24_halfImplicit.h"

using std::chrono::high_resolution_clock;
using namespace d24;

void printVectorOfVectors(const std::vector<std::vector<double>>& vec) {
    for (const auto& inner_vec : vec) {
        for (const auto& element : inner_vec) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv) {
    // Vehicle specification
    std::string vehParamsJSON = (char*)"../../24dof/data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../../24dof/data/json/HMMWV/tmeasy.json";
    std::string susParamsJSON = (char*)"../../24dof/data/json/HMMWV/suspension.json";

    // Construct the solver
    d24SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, susParamsJSON);

    // Set time step
    solver.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    SuspensionState suslf_st;
    SuspensionState susrf_st;
    SuspensionState suslr_st;
    SuspensionState susrr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st);

    // Enable output
    solver.SetOutput("../../24dof/data/output/" + std::string(argv[1]) + "_hmmwv24HiStep.csv", 100);

    double timeStep = solver.GetStep();  // Get the time step
    double endTime = 10;                 // Get the end time
    double t = 0;
    double new_time = 0;

    // Driver inputs
    double throttle = 0;
    double steering = 0;
    double braking = 0;
    // Time the while loop
    auto start = high_resolution_clock::now();

    // Start the loop
    while (t < (endTime - timeStep / 10.)) {
        // Inputs based on time provided externally
        if (t > 0.5) {
            throttle = 0.5;
            steering = 0.1;
            braking = 0;
        } else {
            throttle = 0;
            steering = 0;
            braking = 0;
        }
        new_time = solver.IntegrateStepWithJacobian(t, throttle, steering, braking, true);

// Print the Jacobian state and controls if compiled with DEBUG flag
#ifdef DEBUG
        std::cout << "Jacobian state: " << std::endl;
        printVectorOfVectors(solver.GetJacobianState());
        std::cout << "Jacobian controls: " << std::endl;
        printVectorOfVectors(solver.GetJacobianControls());
        std::cout << std::endl;
#endif

        t = new_time;
    }

    auto stop = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // Print time in milli seconds
    std::cout << "Time taken by function: " << duration.count() / 1000.0 << " milliseconds" << std::endl;
    // Write the output to file (required when solving step-by-step)
    solver.WriteToFile();
}