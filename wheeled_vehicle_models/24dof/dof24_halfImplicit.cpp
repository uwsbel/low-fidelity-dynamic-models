#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof24.h"
#include "dof24_halfImplicit.h"

using namespace d24;
// ======================================================================================================================
// Constructor
d24SolverHalfImplicit::d24SolverHalfImplicit() : m_tend(0), m_step(0.001), m_output(false), m_csv(" ") {
#ifdef USE_OPENMP
    m_num_threads = omp_get_num_procs() / 4;
    // m_num_threads = 6;
    omp_set_num_threads(m_num_threads);
#endif
}
d24SolverHalfImplicit::~d24SolverHalfImplicit() {}
// ======================================================================================================================
/// @brief Construct the the solver using path to vehicle parameters, tire parameters, and driver inputs
/// @param vehicle_params_file Path to the vehicle parameter json file
/// @param tire_params_file Path to the tire parameter json file
/// @param sus_params_file Path to the suspension parameter json file
/// @param driver_inputs_file Path to the driver inputs text file
void d24SolverHalfImplicit::Construct(const std::string& veh_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& sus_params_file,
                                      const std::string& driver_file) {
    // If there is no tire type specified, then use TMeasy
    m_tire_type = TireType::TMeasy;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_tireTM_param, tire_params_file.c_str());
    setSuspensionParamsJSON(m_sus_param, sus_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(m_tireTM_param);

    // Load driver inputs
    LoadDriverData(m_driver_data, driver_inputs_file);

    // Set the final integration time
    m_tend = m_driver_data.back().m_time;
}
/// @brief Construct the the solver using path to vehicle parameters, tire parameters, and driver inputs
/// @param vehicle_params_file Path to the vehicle parameter json file
/// @param tire_params_file Path to the tire parameter json file
/// @param driver_inputs_file Path to the driver inputs text file
/// @param type Tire type to use - TMeasy or TMeasyNR
void d24SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& sus_params_file,
                                      const std::string& driver_inputs_file,
                                      TireType type) {
    // If there is no tire type specified
    m_tire_type = type;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setSuspensionParamsJSON(m_sus_param, sus_params_file.c_str());
    if (m_tire_type == TireType::TMeasy) {
        setTireParamsJSON(m_tireTM_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(m_tireTM_param);
    } else {
        setTireParamsJSON(m_tireTMNr_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(m_tireTMNr_param);
    }

    // Load driver inputs
    LoadDriverData(m_driver_data, driver_inputs_file);

    // Set the final integration time
    m_tend = m_driver_data.back().m_time;
}

// Overload for situations when a controller is used and we don't have a driver data file
void d24SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& sus_params_file) {
    m_tire_type = TireType::TMeasy;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_tireTM_param, tire_params_file.c_str());
    setSuspensionParamsJSON(m_sus_param, sus_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(m_tireTM_param);
}

void d24SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& sus_params_file,
                                      TireType type) {
    // If there is no tire type specified
    m_tire_type = type;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setSuspensionParamsJSON(m_sus_param, sus_params_file.c_str());
    if (m_tire_type == TireType::TMeasy) {
        setTireParamsJSON(m_tireTM_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(m_tireTM_param);
    } else {
        setTireParamsJSON(m_tireTMNr_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(m_tireTMNr_param);
    }
}
// ======================================================================================================================
void d24SolverHalfImplicit::Initialize(d24::VehicleState& vehicle_states,
                                       d24::TMeasyState& tire_states_LF,
                                       d24::TMeasyState& tire_states_RF,
                                       d24::TMeasyState& tire_states_LR,
                                       d24::TMeasyState& tire_states_RR,
                                       d24::SuspensionState& sus_states_LF,
                                       d24::SuspensionState& sus_states_RF,
                                       d24::SuspensionState& sus_states_LR,
                                       d24::SuspensionState& sus_states_RR) {
    m_veh_state = vehicle_states;
    m_tireTMlf_state = tire_states_LF;
    m_tireTMrf_state = tire_states_RF;
    m_tireTMlr_state = tire_states_LR;
    m_tireTMrr_state = tire_states_RR;
    m_suslf_state = sus_states_LF;
    m_susrf_state = sus_states_RF;
    m_suslr_state = sus_states_LR;
    m_susrr_state = sus_states_RR;

    d24::initializeTireSus(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                           m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_sus_param,
                           m_tireTM_param);

    // Size the jacobian matrices - size relies on the torque converter bool
    m_num_controls = 2;
    if (m_veh_param._tcbool) {
        m_num_states = 21;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    } else {
        m_num_states = 20;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    }
}

void d24SolverHalfImplicit::Initialize(d24::VehicleState& vehicle_states,
                                       d24::TMeasyNrState& tire_states_LF,
                                       d24::TMeasyNrState& tire_states_RF,
                                       d24::TMeasyNrState& tire_states_LR,
                                       d24::TMeasyNrState& tire_states_RR,
                                       d24::SuspensionState& sus_states_LF,
                                       d24::SuspensionState& sus_states_RF,
                                       d24::SuspensionState& sus_states_LR,
                                       d24::SuspensionState& sus_states_RR) {
    m_veh_state = vehicle_states;
    m_tireTMNrlf_state = tire_states_LF;
    m_tireTMNrrf_state = tire_states_RF;
    m_tireTMNrlr_state = tire_states_LR;
    m_tireTMNrrr_state = tire_states_RR;
    m_suslf_state = sus_states_LF;
    m_susrf_state = sus_states_RF;
    m_suslr_state = sus_states_LR;
    m_susrr_state = sus_states_RR;

    // Size the jacobian matrices - size relies on the torque converter bool
    m_num_controls = 2;
    if (m_veh_param._tcbool) {
        m_num_states = 13;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    } else {
        m_num_states = 12;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    }
}

// ======================================================================================================================
/// @brief Sets the path for the output file
/// @param output_file string with full path with extension
void d24SolverHalfImplicit::SetOutput(const std::string& output_file, double output_freq) {
    m_output = true;
    m_output_file = output_file;
    m_timeStepsStored = 0;
    m_dtout = 1.0 / output_freq;
}

// ======================================================================================================================
/// @brief Solve the system of equations by calling the integrate function
void d24SolverHalfImplicit::Solve() {
    assert(!m_driver_data.empty() && "No controls provided, please use construct to pass path to driver inputs");

    // For now just integrate to final time
    Integrate();
}
// ======================================================================================================================

/// @brief Integrate the system of equations using the half implicit method - Calls the RHS function at each time step
void d24SolverHalfImplicit::Integrate() {
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

        if (m_tire_type == TireType::TMeasy) {
            m_tireTMlf_state._xe += m_tireTMlf_state._xedot * m_step;
            m_tireTMlf_state._ye += m_tireTMlf_state._yedot * m_step;
            m_tireTMlf_state._xt += m_tireTMlf_state._dxt * m_step;
            m_tireTMlf_state._omega += m_tireTMlf_state._dOmega * m_step;
            // RF
            m_tireTMrf_state._xe += m_tireTMrf_state._xedot * m_step;
            m_tireTMrf_state._ye += m_tireTMrf_state._yedot * m_step;
            m_tireTMrf_state._xt += m_tireTMrf_state._dxt * m_step;
            m_tireTMrf_state._omega += m_tireTMrf_state._dOmega * m_step;
            // LR
            m_tireTMlr_state._xe += m_tireTMlr_state._xedot * m_step;
            m_tireTMlr_state._ye += m_tireTMlr_state._yedot * m_step;
            m_tireTMlr_state._xt += m_tireTMlr_state._dxt * m_step;
            m_tireTMlr_state._omega += m_tireTMlr_state._dOmega * m_step;
            // RR
            m_tireTMrr_state._xe += m_tireTMrr_state._xedot * m_step;
            m_tireTMrr_state._ye += m_tireTMrr_state._yedot * m_step;
            m_tireTMrr_state._xt += m_tireTMrr_state._dxt * m_step;
            m_tireTMrr_state._omega += m_tireTMrr_state._dOmega * m_step;
        } else {
            // LF
            m_tireTMlf_state._xt += m_tireTMlf_state._dxt * m_step;
            m_tireTMNrlf_state._omega += m_tireTMNrlf_state._dOmega * m_step;
            // RF
            m_tireTMrf_state._xt += m_tireTMrf_state._dxt * m_step;
            m_tireTMNrrf_state._omega += m_tireTMNrrf_state._dOmega * m_step;
            // LR
            m_tireTMlr_state._xt += m_tireTMlr_state._dxt * m_step;
            m_tireTMNrlr_state._omega += m_tireTMNrlr_state._dOmega * m_step;
            // RR
            m_tireTMrr_state._xt += m_tireTMrr_state._dxt * m_step;
            m_tireTMNrrr_state._omega += m_tireTMNrrr_state._dOmega * m_step;
        }

        // Integrate the suspension states
        m_suslf_state._wu += m_suslf_state._dwu * m_step;
        m_suslf_state._xs += m_suslf_state._dxs * m_step;

        m_susrf_state._wu += m_susrf_state._dwu * m_step;
        m_susrf_state._xs += m_susrf_state._dxs * m_step;

        m_suslr_state._wu += m_suslr_state._dwu * m_step;
        m_suslr_state._xs += m_suslr_state._dxs * m_step;

        m_susrr_state._wu += m_susrr_state._dwu * m_step;
        m_susrr_state._xs += m_susrr_state._dxs * m_step;

        // Integrate the vehicle states
        if (m_veh_param._tcbool) {
            m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
        }
        m_veh_state._u += m_veh_state._udot * m_step;
        m_veh_state._v += m_veh_state._vdot * m_step;
        m_veh_state._w += m_veh_state._wdot * m_step;
        m_veh_state._wx += m_veh_state._wx_dot * m_step;
        m_veh_state._wy += m_veh_state._wy_dot * m_step;
        m_veh_state._wz += m_veh_state._wz_dot * m_step;

        // We have to recompute this here so that we are doing the half implicit
        // Now to integrate the cardan angles, we first write equations for them
        m_veh_state._dtheta =
            m_veh_state._wy * std::cos(m_veh_state._phi) - m_veh_state._wz * std::sin(m_veh_state._phi);
        m_veh_state._dpsi =
            (m_veh_state._wy * std::sin(m_veh_state._phi) / std::cos(m_veh_state._theta)) +
            (m_veh_state._wz * std::cos(m_veh_state._phi) / std::cos(m_veh_state._theta));  // Trouble when theta is 90?
        m_veh_state._dphi = m_veh_state._wx +
                            m_veh_state._wy * std::sin(m_veh_state._phi) * std::tan(m_veh_state._theta) +
                            m_veh_state._wz * std::cos(m_veh_state._phi) * std::tan(m_veh_state._theta);

        m_veh_state._theta += m_veh_state._dtheta * m_step;
        m_veh_state._psi += m_veh_state._dpsi * m_step;
        m_veh_state._phi += m_veh_state._dphi * m_step;

        // This computation is also repeated because of half implicit
        // Update global position
        m_veh_state._x +=
            (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
        m_veh_state._y +=
            (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;

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

/// @brief Computes the RHS of all the ODEs (tire velocities, suspension velocities and accelerations chassis
/// accelerations)
/// @param t Current time
void d24SolverHalfImplicit::rhsFun(double t) {
    // Get controls at the current timeStep
    auto controls = GetDriverInput(t, m_driver_data);

    if (m_tire_type == TireType::TMeasy) {
        vehToSusTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, controls.m_steering);
        vehToTireTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                           m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param,
                           controls.m_steering);
        // Tire velocities
        computeTireRHS(m_tireTMlf_state, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMrf_state, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMlr_state, m_tireTM_param, m_veh_param, 0);  // No rear steering
        computeTireRHS(m_tireTMrr_state, m_tireTM_param, m_veh_param, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state,
                                       m_tireTMrr_state, m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state);
#ifdef DEBUG
        M_DEBUG_LF_TIRE_FX = m_tireTMlf_state._fx;
        M_DEBUG_RF_TIRE_FX = m_tireTMrf_state._fx;
        M_DEBUG_LR_TIRE_FX = m_tireTMlr_state._fx;
        M_DEBUG_RR_TIRE_FX = m_tireTMrr_state._fx;

        M_DEBUG_LF_TIRE_FY = m_tireTMlf_state._fy;
        M_DEBUG_RF_TIRE_FY = m_tireTMrf_state._fy;
        M_DEBUG_LR_TIRE_FY = m_tireTMlr_state._fy;
        M_DEBUG_RR_TIRE_FY = m_tireTMrr_state._fy;

        M_DEBUG_LF_TIRE_FZ = m_tireTMlf_state._fz;
        M_DEBUG_RF_TIRE_FZ = m_tireTMrf_state._fz;
        M_DEBUG_LR_TIRE_FZ = m_tireTMlr_state._fz;
        M_DEBUG_RR_TIRE_FZ = m_tireTMrr_state._fz;
#endif
        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                             m_veh_param, m_tireTM_param, controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                           m_veh_param, controls.m_steering);
        // Suspension velocities
        computeSuspensionRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                             m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_sus_param);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                                m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_tireTM_param,
                                m_sus_param);

        // Compute the chassis accelerations
        computeVehicleRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_tireTM_param,
                          m_sus_param);
    }
}