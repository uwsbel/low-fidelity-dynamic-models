// =============================================================================
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// Run with ./accTest to reproduce data for the Acceleration test case shown in paper
// To generate the plots run accTestPlot.py
//
// =============================================================================

#include <numeric>
#include <algorithm>
#include <iterator>

#include "dof11_halfImplicit.h"
#include "dof18_halfImplicit.h"
#include "dof24_halfImplicit.h"

int main(int argc, char** argv) {
    // Driver inputs - Common for all models
    std::string driver_file = "../../../wheeled_vehicle_models/18dof/data/input/acc10.txt";
    // =============================================================================
    // 11 DoF Model
    // =============================================================================

    // Vehicle specification
    std::string vehParamsJSON11 = (char*)"../../../wheeled_vehicle_models/11dof/data/json/Sedan/vehicle.json";
    std::string tireParamsJSON11 = (char*)"../../../wheeled_vehicle_models/11dof/data/json/Sedan/tmeasyNr.json";

    // Construct the solver
    d11SolverHalfImplicit solver11;
    // Since the TMeasyNr tire is used, the TireType must be set
    d11::TireType type11 = d11::TireType::TMeasyNr;
    solver11.Construct(vehParamsJSON11, tireParamsJSON11, driver_file, type11);

    // Set time step
    solver11.SetTimeStep(1e-3);

    // Initialize solver (set initial conditions)
    d11::VehicleState veh_st11;
    d11::TMeasyNrState tiref_st11;
    d11::TMeasyNrState tirer_st11;

    solver11.Initialize(veh_st11, tiref_st11, tirer_st11);

    // Enable output
    solver11.SetOutput("../data/LFDM/acc_sedan11Hi.csv", 100);
    solver11.Solve();

    // =============================================================================
    // 18 DoF Model
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

    // Initialize solver (set initial conditions)
    d18::VehicleState veh_st18;
    d18::TMeasyNrState tirelf_st18;
    d18::TMeasyNrState tirerf_st18;
    d18::TMeasyNrState tirelr_st18;
    d18::TMeasyNrState tirerr_st18;
    solver18.Initialize(veh_st18, tirelf_st18, tirerf_st18, tirelr_st18, tirerr_st18);

    // Enable output
    solver18.SetOutput("../data/LFDM/acc_sedan18Hi.csv", 100);
    solver18.Solve();

    // =============================================================================
    // 24 DoF Model
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

    // Initialize solver (set initial conditions)
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

    solver24.SetOutput("../data/LFDM/acc_sedan24Hi.csv", 100);
    solver24.Solve();
}