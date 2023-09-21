#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof18.h"
#include "dof18_halfImplicit.h"

using namespace d18;
// ======================================================================================================================
d18SolverHalfImplicit::d18SolverHalfImplicit() : m_tend(0), m_step(0.001), m_output(false), m_csv(" ") {}
d18SolverHalfImplicit::~d18SolverHalfImplicit() {}
// ======================================================================================================================

/// @brief Construct the the solver using path to vehicle parameters, tire parameters, and driver inputs
/// @param vehicle_params_file Path to the vehicle parameter json file
/// @param tire_params_file Path to the tire parameter json file
/// @param driver_inputs_file Path to the driver inputs text file
void d18SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& driver_inputs_file) {
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_tire_param, tire_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(m_tire_param);

    // Load driver inputs
    LoadDriverData(m_driver_data, driver_inputs_file);

    // Set the final integration time
    m_tend = m_driver_data.back().m_time;
}

// ======================================================================================================================

/// @brief Initialize vehicle and tire states in case something other than non zero is needed
/// @param vehicle_states
/// @param tire_states_LF
/// @param tire_states_RF
/// @param tire_states_LR
/// @param tire_states_RR
void d18SolverHalfImplicit::Initialize(d18::VehicleState& vehicle_states,
                                       d18::TMeasyState& tire_states_LF,
                                       d18::TMeasyState& tire_states_RF,
                                       d18::TMeasyState& tire_states_LR,
                                       d18::TMeasyState& tire_states_RR) {
    m_veh_state = vehicle_states;
    m_tirelf_state = tire_states_LF;
    m_tirerf_state = tire_states_RF;
    m_tirelr_state = tire_states_LR;
    m_tirerr_state = tire_states_RR;
}

// ======================================================================================================================

/// @brief Sets the path for the output file
/// @param output_file string with full path with extension
void d18SolverHalfImplicit::SetOutput(const std::string& output_file, double output_freq) {
    m_output = true;
    m_output_file = output_file;
    m_timeStepsStored = 0;
    m_dtout = 1.0 / output_freq;
}

// ======================================================================================================================

/// @brief Solve the system of equations by calling the integrate function
void d18SolverHalfImplicit::Solve() {
    // For now just integrate to final time
    Integrate();
}

// ======================================================================================================================

/// @brief Integrate the system of equations using the half implicit method - Calls the RHS function at each time step
void d18SolverHalfImplicit::Integrate() {
    double t = 0;
    // Create output writer
    if (m_output) {
        Write(t);
        m_timeStepsStored++;
    }

    // Integrate to final time by repeatedly calling the RHS function
    while (t < (m_tend - m_step / 10.)) {
        // Call RHS to get all accelerations
        rhsFun(t);

        // Integrate according to half implicit method for second order states
        // Integrate according to explicit method for first order states

        // First the tire states
        // LF
        m_tirelf_state._xe += m_tirelf_state._xedot * m_step;
        m_tirelf_state._ye += m_tirelf_state._yedot * m_step;
        m_tirelf_state._omega += m_tirelf_state._dOmega * m_step;
        // RF
        m_tirerf_state._xe += m_tirerf_state._xedot * m_step;
        m_tirerf_state._ye += m_tirerf_state._yedot * m_step;
        m_tirerf_state._omega += m_tirerf_state._dOmega * m_step;
        // LR
        m_tirelr_state._xe += m_tirelr_state._xedot * m_step;
        m_tirelr_state._ye += m_tirelr_state._yedot * m_step;
        m_tirelr_state._omega += m_tirelr_state._dOmega * m_step;
        // RR
        m_tirerr_state._xe += m_tirerr_state._xedot * m_step;
        m_tirerr_state._ye += m_tirerr_state._yedot * m_step;
        m_tirerr_state._omega += m_tirerr_state._dOmega * m_step;

        // Now the vehicle states
        if (m_veh_param._tcbool) {
            m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
        }

        // Integrate velocity level first
        m_veh_state._u += m_veh_state._udot * m_step;
        m_veh_state._v += m_veh_state._vdot * m_step;
        m_veh_state._wx += m_veh_state._wxdot * m_step;
        m_veh_state._wz += m_veh_state._wzdot * m_step;

        // Integrate position level next
        m_veh_state._x +=
            (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
        m_veh_state._y +=
            (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
        m_veh_state._psi += m_veh_state._wz * m_step;
        m_veh_state._phi += m_veh_state._wx * m_step;

        // Update time
        t += m_step;

        // Write the output
        if (m_output) {
            if (std::abs(t - m_timeStepsStored * m_dtout) < 1e-7) {
                Write(t);
                m_timeStepsStored++;
            }
        }
    }
    if (m_output) {
        WriteToFile();
    }
}
// ======================================================================================================================

/// @brief Function call to integrate by just a single time step. This function will always integrate to the t + m_step
/// where m_step is set using the SetTimeStep function.
/// @param t current time
void d18SolverHalfImplicit::IntegrateStep(double t, double throttle, double steering, double braking) {
    // Store header and first time step
    if (m_output && t < m_step) {
        Write(t);
        m_timeStepsStored++;
    }

    DriverInput controls(t, steering, throttle, braking);
    // Call the RHS function
    rhsFun(t, controls);

    // Integrate according to half implicit method for second order states
    // Integrate according to explicit method for first order states

    // First the tire states
    // LF
    m_tirelf_state._xe += m_tirelf_state._xedot * m_step;
    m_tirelf_state._ye += m_tirelf_state._yedot * m_step;
    m_tirelf_state._omega += m_tirelf_state._dOmega * m_step;
    // RF
    m_tirerf_state._xe += m_tirerf_state._xedot * m_step;
    m_tirerf_state._ye += m_tirerf_state._yedot * m_step;
    m_tirerf_state._omega += m_tirerf_state._dOmega * m_step;
    // LR
    m_tirelr_state._xe += m_tirelr_state._xedot * m_step;
    m_tirelr_state._ye += m_tirelr_state._yedot * m_step;
    m_tirelr_state._omega += m_tirelr_state._dOmega * m_step;
    // RR
    m_tirerr_state._xe += m_tirerr_state._xedot * m_step;
    m_tirerr_state._ye += m_tirerr_state._yedot * m_step;
    m_tirerr_state._omega += m_tirerr_state._dOmega * m_step;

    // Now the vehicle states
    if (m_veh_param._tcbool) {
        m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
    }

    // Integrate velocity level first
    m_veh_state._u += m_veh_state._udot * m_step;
    m_veh_state._v += m_veh_state._vdot * m_step;
    m_veh_state._wx += m_veh_state._wxdot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // Integrate position level next
    m_veh_state._x +=
        (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
    m_veh_state._y +=
        (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
    m_veh_state._psi += m_veh_state._wz * m_step;
    m_veh_state._phi += m_veh_state._wx * m_step;

    // Write the output
    if (m_output) {
        if (std::abs(t - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(t);
            m_timeStepsStored++;
        }
    }
}

// ======================================================================================================================

/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations)
/// @param t Current time
void d18SolverHalfImplicit::rhsFun(double t) {
    // Get controls at the current timeStep
    auto controls = GetDriverInput(t, m_driver_data);

    // Calculate tire vertical loads
    std::vector<double> loads(4, 0);
    computeTireLoads(loads, m_veh_state, m_veh_param, m_tire_param);

    // Transform from vehicle frame to the tire frame
    vehToTireTransform(m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_state, loads, m_veh_param,
                       controls.m_steering);

    // Tire velocities using TMEasy tire
    computeTireRHS(m_tirelf_state, m_tire_param, m_veh_param, controls.m_steering);
    computeTireRHS(m_tirerf_state, m_tire_param, m_veh_param, controls.m_steering);
    computeTireRHS(m_tirelr_state, m_tire_param, m_veh_param, 0);  // No rear steering
    computeTireRHS(m_tirerr_state, m_tire_param, m_veh_param, 0);  // No rear steering

    // Powertrain dynamics
    computePowertrainRHS(m_veh_state, m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_param,
                         m_tire_param, controls);

    // Vehicle dynamics
    tireToVehTransform(m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_state, m_veh_param,
                       controls.m_steering);
    std::vector<double> fx = {m_tirelf_state._fx, m_tirerf_state._fx, m_tirelr_state._fx, m_tirerr_state._fx};
    std::vector<double> fy = {m_tirelf_state._fy, m_tirerf_state._fy, m_tirelr_state._fy, m_tirerr_state._fy};
    computeVehRHS(m_veh_state, m_veh_param, fx, fy);
}

// ======================================================================================================================

void d18SolverHalfImplicit::rhsFun(double t, DriverInput& controls) {
    // Calculate tire vertical loads
    std::vector<double> loads(4, 0);
    computeTireLoads(loads, m_veh_state, m_veh_param, m_tire_param);

    // Transform from vehicle frame to the tire frame
    vehToTireTransform(m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_state, loads, m_veh_param,
                       controls.m_steering);

    // Tire velocities using TMEasy tire
    computeTireRHS(m_tirelf_state, m_tire_param, m_veh_param, controls.m_steering);
    computeTireRHS(m_tirerf_state, m_tire_param, m_veh_param, controls.m_steering);
    computeTireRHS(m_tirelr_state, m_tire_param, m_veh_param, 0);  // No rear steering
    computeTireRHS(m_tirerr_state, m_tire_param, m_veh_param, 0);  // No rear steering

    // Powertrain dynamics
    computePowertrainRHS(m_veh_state, m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_param,
                         m_tire_param, controls);

    // Vehicle dynamics
    tireToVehTransform(m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_state, m_veh_param,
                       controls.m_steering);
    std::vector<double> fx = {m_tirelf_state._fx, m_tirerf_state._fx, m_tirelr_state._fx, m_tirerr_state._fx};
    std::vector<double> fy = {m_tirelf_state._fy, m_tirerf_state._fy, m_tirelr_state._fy, m_tirerr_state._fy};
    computeVehRHS(m_veh_state, m_veh_param, fx, fy);
}

// ======================================================================================================================
void d18SolverHalfImplicit::Write(double t) {
    // If we are in initial time step, write the header
    if (t < m_step) {
        m_csv << "time";
        m_csv << "x";
        m_csv << "y";
        m_csv << "vx";
        m_csv << "vy";
        m_csv << "ax";
        m_csv << "ay";
        m_csv << "roll";
        m_csv << "yaw";
        m_csv << "roll_rate";
        m_csv << "yaw_rate";
        m_csv << "wlf";
        m_csv << "wrf";
        m_csv << "wlr";
        m_csv << "wrr";
        m_csv << "sp_tor";
        m_csv << "current_gear";
        m_csv << "engine_omega";
        m_csv << "tiredef_lf";
        m_csv << "tiredef_rf";
        m_csv << "tiredef_lr";
        m_csv << "tiredef_rr";
        m_csv << std::endl;

        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << std::endl;
        return;
    }

    m_csv << t;
    m_csv << m_veh_state._x;
    m_csv << m_veh_state._y;
    m_csv << m_veh_state._u;
    m_csv << m_veh_state._v;
    m_csv << m_veh_state._udot;
    m_csv << m_veh_state._vdot;
    m_csv << m_veh_state._phi;
    m_csv << m_veh_state._psi;
    m_csv << m_veh_state._wx;
    m_csv << m_veh_state._wz;
    m_csv << m_tirelf_state._omega;
    m_csv << m_tirerf_state._omega;
    m_csv << m_tirelr_state._omega;
    m_csv << m_tirerr_state._omega;
    m_csv << m_veh_state._tor / 4.;
    m_csv << m_veh_state._current_gr + 1;
    m_csv << m_veh_state._crankOmega;
    m_csv << m_tirelf_state._xe;
    m_csv << m_tirerf_state._xe;
    m_csv << m_tirelr_state._xe;
    m_csv << m_tirerr_state._xe;
    m_csv << std::endl;
}

// ======================================================================================================================

void d18SolverHalfImplicit::WriteToFile() {
    if (!m_output) {
        std::cout << "No output file specified. Call SetOutput() before calling WriteToFile()" << std::endl;
        return;
    }
    m_csv.write_to_file(m_output_file);
    m_csv.clearData();
    m_timeStepsStored = 0;
}