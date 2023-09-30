#include <numeric>
#include <algorithm>
#include <iterator>
#include <chrono>

#include "dof18_halfImplicit.h"

using std::chrono::high_resolution_clock;
using namespace d18;

void printVectorOfVectors(const std::vector<std::vector<double>>& vec) {
    for (const auto& inner_vec : vec) {
        for (const auto& element : inner_vec) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/" + std::string(argv[1]) + ".txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";

    // Construct the solver
    d18SolverHalfImplicit solver;
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

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
    solver.SetOutput("../data/output/" + std::string(argv[1]) + "_hmmwv18HiStep.csv", 100);

    double timeStep = solver.GetStep();
    double endTime = solver.GetEndT();
    double t = 0;
    double new_time = 0;
    DriverData driver_data = solver.GetDriverData();  // Get the driver data associated with input file

    // Time the while loop
    auto start = high_resolution_clock::now();

    // Start the loop
    while (t < (endTime - timeStep / 10.)) {
        auto controls = GetDriverInput(t, driver_data);  // Get the controls for the current time
        new_time =
            solver.IntegrateStepWithJacobian(t, controls.m_throttle, controls.m_steering, controls.m_braking, true);

// Print the Jacobian state and controls
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
}