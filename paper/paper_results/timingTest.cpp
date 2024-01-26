// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// Run with ./timingTest to reproduce data for the simulation speed comparison case shown in paper
// Benchmark done without writing output file
//
// =============================================================================

#include <numeric>
#include <algorithm>
#include <cmath>
#include <iterator>

#include "dof11_halfImplicit.h"
#include "dof18_halfImplicit.h"
#include "dof24_halfImplicit.h"

// Timing libraries
#include <chrono>

int main(int argc, char** argv) {
    // Driver inputs - Common for all models
    std::string driver_file = "../../../wheeled_vehicle_models/18dof/data/input/acc10.txt";

    // =============================================================================
    // Construct the Solvers
    // =============================================================================

    // Vehicle specification
    std::string vehParamsJSON11 = (char*)"../../../wheeled_vehicle_models/11dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON11 = (char*)"../../../wheeled_vehicle_models/11dof/data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    d11SolverHalfImplicit solver11;
    // Since the TMeasyNr tire is used, the TireType must be set
    d11::TireType type11 = d11::TireType::TMeasyNr;
    solver11.Construct(vehParamsJSON11, tireParamsJSON11, driver_file, type11);
    solver11.SetTimeStep(1e-3);

    // =============================================================================

    // Vehicle specification
    std::string vehParamsJSON18 = (char*)"../../../wheeled_vehicle_models/18dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON18 = (char*)"../../../wheeled_vehicle_models/18dof/data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    d18SolverHalfImplicit solver18;
    // Since the TMeasyNr tire is used, the TireType must be set
    d18::TireType type18 = d18::TireType::TMeasyNr;
    solver18.Construct(vehParamsJSON18, tireParamsJSON18, driver_file, type18);

    // Set time step
    solver18.SetTimeStep(1e-3);

    // =============================================================================

    // Vehicle specification
    std::string vehParamsJSON24 = (char*)"../../../wheeled_vehicle_models/24dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON24 = (char*)"../../../wheeled_vehicle_models/24dof/data/json/Sedan/tmeasyNr.json";
    std::string susParamsJSON24 = (char*)"../../../wheeled_vehicle_models/24dof/data/json/Sedan/suspension.json";

    // Construct the solver
    d24SolverHalfImplicit solver24;
    // Since the TMeasyNr tire is used, the TireType must be set
    d24::TireType type24 = d24::TireType::TMeasyNr;
    solver24.Construct(vehParamsJSON24, tireParamsJSON24, susParamsJSON24, driver_file, type24);

    // Set time step
    solver24.SetTimeStep(1e-3);

    // =============================================================================
    // Loop
    // =============================================================================

    const int num_trials = 100;
    std::vector<double> elapsed_times11;
    std::vector<double> elapsed_times18;
    std::vector<double> elapsed_times24;

    elapsed_times11.reserve(num_trials);
    elapsed_times18.reserve(num_trials);
    elapsed_times24.reserve(num_trials);

    for (int i = 0; i < num_trials; ++i) {
        // =============================================================================
        // 11 DoF Model
        // =============================================================================
        d11::VehicleState veh_st11;
        d11::TMeasyNrState tiref_st11;
        d11::TMeasyNrState tirer_st11;

        solver11.Initialize(veh_st11, tiref_st11, tirer_st11);

        // Time the Solve() function
        auto start = std::chrono::high_resolution_clock::now();
        solver11.Solve();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        elapsed_times11.push_back(elapsed.count());

        // =============================================================================
        // 18 DoF Model
        // =============================================================================
        d18::VehicleState veh_st18;
        d18::TMeasyNrState tirelf_st18;
        d18::TMeasyNrState tirerf_st18;
        d18::TMeasyNrState tirelr_st18;
        d18::TMeasyNrState tirerr_st18;
        solver18.Initialize(veh_st18, tirelf_st18, tirerf_st18, tirelr_st18, tirerr_st18);

        // Time
        start = std::chrono::high_resolution_clock::now();
        solver18.Solve();
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        elapsed_times18.push_back(elapsed.count());

        // =============================================================================
        // 24 DoF Model
        // =============================================================================
        d24::VehicleState veh_st24;
        d24::TMeasyNrState tirelf_st24;
        d24::TMeasyNrState tirerf_st24;
        d24::TMeasyNrState tirelr_st24;
        d24::TMeasyNrState tirerr_st24;
        d24::SuspensionState suslf_st24;
        d24::SuspensionState susrf_st24;
        d24::SuspensionState suslr_st24;
        d24::SuspensionState susrr_st24;
        solver24.Initialize(veh_st24, tirelf_st24, tirerf_st24, tirelr_st24, tirerr_st24, suslf_st24, susrf_st24,
                            suslr_st24, susrr_st24);

        start = std::chrono::high_resolution_clock::now();
        solver24.Solve();
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        elapsed_times24.push_back(elapsed.count());
    }

    // =============================================================================
    // 11 DoF Model stats
    // =============================================================================

    // Calculate mean
    double mean11 = std::accumulate(elapsed_times11.begin(), elapsed_times11.end(), 0.0) / num_trials;

    // Calculate standard deviation
    double variance11 = 0.0;
    for (double time : elapsed_times11) {
        variance11 += std::pow(time - mean11, 2);
    }
    variance11 /= num_trials;
    double stddev11 = std::sqrt(variance11);

    std::cout << "11 DoF model mean time: " << mean11 << " seconds" << std::endl;
    std::cout << "11 DoF model standard deviation: " << stddev11 << " seconds" << std::endl;
    double simulated_time = 10.0;
    double rtf = mean11 / simulated_time;
    std::cout << "11 DoF model RTF: " << rtf << std::endl;
    double times_faster = 1.0 / rtf;
    std::cout << "11 DoF model is " << times_faster << " faster than real-time" << std::endl;

    // =============================================================================
    // 18 DoF Model stats
    // =============================================================================

    // Calculate mean
    double mean18 = std::accumulate(elapsed_times18.begin(), elapsed_times18.end(), 0.0) / num_trials;

    // Calculate standard deviation
    double variance18 = 0.0;
    for (double time : elapsed_times18) {
        variance18 += std::pow(time - mean18, 2);
    }
    variance18 /= num_trials;
    double stddev18 = std::sqrt(variance18);

    std::cout << "18 DoF model mean time: " << mean18 << " seconds" << std::endl;
    std::cout << "18 DoF model standard deviation: " << stddev18 << " seconds" << std::endl;
    rtf = mean18 / simulated_time;
    std::cout << "18 DoF model RTF: " << rtf << std::endl;
    times_faster = 1.0 / rtf;
    std::cout << "18 DoF model is " << times_faster << " faster than real-time" << std::endl;

    // =============================================================================
    // 24 DoF Model Stats
    // =============================================================================

    // Calculate mean
    double mean24 = std::accumulate(elapsed_times24.begin(), elapsed_times24.end(), 0.0) / num_trials;

    // Calculate standard deviation
    double variance24 = 0.0;
    for (double time : elapsed_times24) {
        variance24 += std::pow(time - mean24, 2);
    }
    variance24 /= num_trials;
    double stddev24 = std::sqrt(variance24);

    std::cout << "24 DoF model mean time: " << mean24 << " seconds" << std::endl;
    std::cout << "24 DoF model standard deviation: " << stddev24 << " seconds" << std::endl;
    rtf = mean24 / simulated_time;
    std::cout << "24 DoF model RTF: " << rtf << std::endl;
    times_faster = 1.0 / rtf;
    std::cout << "24 DoF model is " << times_faster << " faster than real-time" << std::endl;
}