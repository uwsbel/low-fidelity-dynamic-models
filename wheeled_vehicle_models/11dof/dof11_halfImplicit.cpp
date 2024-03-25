#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof11.h"
#include "dof11_halfImplicit.h"

using namespace d11;
// ======================================================================================================================
d11SolverHalfImplicit::d11SolverHalfImplicit() : m_tend(0), m_step(0.001), m_output(false), m_csv(" ") {
#ifdef USE_OPENMP
    m_num_threads = omp_get_num_procs() / 4;
    // m_num_threads = 6;
    omp_set_num_threads(m_num_threads);
#endif
}
d11SolverHalfImplicit::~d11SolverHalfImplicit() {}
// ======================================================================================================================

/// @brief Construct the the solver using path to vehicle parameters, tire parameters, and driver inputs
/// @param vehicle_params_file Path to the vehicle parameter json file
/// @param tire_params_file Path to the tire parameter json file
/// @param driver_inputs_file Path to the driver inputs text file
void d11SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& driver_inputs_file) {
    m_tire_type = TireType::TMeasy;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_tireTM_param, tire_params_file.c_str());
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
void d11SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& driver_inputs_file,
                                      TireType type) {
    // If there is no tire type specified
    m_tire_type = type;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
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
void d11SolverHalfImplicit::Construct(const std::string& vehicle_params_file, const std::string& tire_params_file) {
    m_tire_type = TireType::TMeasy;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_tireTM_param, tire_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(m_tireTM_param);
}

void d11SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      TireType type) {
    // If there is no tire type specified
    m_tire_type = type;
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
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

/// @brief Initialize vehicle and tire states in case something other than non zero is needed - has to be called after
/// construct for now
/// @param vehicle_states
/// @param tire_states_F (front)
/// @param tire_states_R (rear)
void d11SolverHalfImplicit::Initialize(d11::VehicleState& vehicle_states,
                                       d11::TMeasyState& tire_states_F,
                                       d11::TMeasyState& tire_states_R) {
    m_veh_state = vehicle_states;
    m_tireTMf_state = tire_states_F;
    m_tireTMr_state = tire_states_R;

    // Size the jacobian matrices - size relies on the torque converter bool
    m_num_controls = 2;
    if (m_veh_param._tcbool) {
        m_num_states = 13;
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    } else {
        m_num_states = 12;
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    }
}

void d11SolverHalfImplicit::Initialize(d11::VehicleState& vehicle_states,
                                       d11::TMeasyNrState& tire_states_F,
                                       d11::TMeasyNrState& tire_states_R) {
    m_veh_state = vehicle_states;
    m_tireTMNrf_state = tire_states_F;
    m_tireTMNrr_state = tire_states_R;

    // Size the jacobian matrices - size relies on the torque converter bool
    m_num_controls = 2;
    if (m_veh_param._tcbool) {
        m_num_states = 9;
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    } else {
        m_num_states = 8;
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    }
}

// ======================================================================================================================

/// @brief Sets the path for the output file
/// @param output_file string with full path with extension
void d11SolverHalfImplicit::SetOutput(const std::string& output_file, double output_freq) {
    m_output = true;
    m_output_file = output_file;
    m_timeStepsStored = 0;
    m_dtout = 1.0 / output_freq;
}

// ======================================================================================================================

/// @brief Solve the system of equations by calling the integrate function
void d11SolverHalfImplicit::Solve() {
    assert(!m_driver_data.empty() && "No controls provided, please use construct to pass path to driver inputs");

    // For now just integrate to final time
    Integrate();
}

// ======================================================================================================================

/// @brief Integrate the system of equations using the half implicit method - Calls the RHS function at each time step
void d11SolverHalfImplicit::Integrate() {
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
        if (m_tire_type == TireType::TMeasy) {
            // First the tire states
            // F
            m_tireTMf_state._xe += m_tireTMf_state._xedot * m_step;
            m_tireTMf_state._ye += m_tireTMf_state._yedot * m_step;
            m_tireTMf_state._omega += m_tireTMf_state._dOmega * m_step;
            // R
            m_tireTMr_state._xe += m_tireTMr_state._xedot * m_step;
            m_tireTMr_state._ye += m_tireTMr_state._yedot * m_step;
            m_tireTMr_state._omega += m_tireTMr_state._dOmega * m_step;
        } else {  // TMeasyNr only has omega states
            // F
            m_tireTMNrf_state._omega += m_tireTMNrf_state._dOmega * m_step;
            // R
            m_tireTMNrr_state._omega += m_tireTMNrr_state._dOmega * m_step;
        }

        // Now the vehicle states
        if (m_veh_param._tcbool) {
            m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
        }

        // Integrate velocity level first
        m_veh_state._u += m_veh_state._udot * m_step;
        m_veh_state._v += m_veh_state._vdot * m_step;
        m_veh_state._wz += m_veh_state._wzdot * m_step;

        // Integrate position level next
        m_veh_state._x +=
            (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
        m_veh_state._y +=
            (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
        m_veh_state._psi += m_veh_state._wz * m_step;

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
/// @param throttle throttle input
/// @param steering steering input
/// @param braking braking input
/// @return t + m_step
double d11SolverHalfImplicit::IntegrateStep(double t, double throttle, double steering, double braking) {
    // Store header and first time step
    if (m_output && (t < m_step)) {
        Write(t);
        m_timeStepsStored++;
    }

    DriverInput controls(t, steering, throttle, braking);
    // Call the RHS function
    rhsFun(t, controls);

    // Integrate according to half implicit method for second order states
    // Integrate according to explicit method for first order states
    if (m_tire_type == TireType::TMeasy) {
        // First the tire states
        // F
        m_tireTMf_state._xe += m_tireTMf_state._xedot * m_step;
        m_tireTMf_state._ye += m_tireTMf_state._yedot * m_step;
        m_tireTMf_state._omega += m_tireTMf_state._dOmega * m_step;
        // R
        m_tireTMr_state._xe += m_tireTMr_state._xedot * m_step;
        m_tireTMr_state._ye += m_tireTMr_state._yedot * m_step;
        m_tireTMr_state._omega += m_tireTMr_state._dOmega * m_step;
    } else {
        // F
        m_tireTMNrf_state._omega += m_tireTMNrf_state._dOmega * m_step;
        // R
        m_tireTMNrr_state._omega += m_tireTMNrr_state._dOmega * m_step;
    }

    // Now the vehicle states
    if (m_veh_param._tcbool) {
        m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
    }

    // Integrate velocity level first
    m_veh_state._u += m_veh_state._udot * m_step;
    m_veh_state._v += m_veh_state._vdot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // Integrate position level next
    m_veh_state._x +=
        (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
    m_veh_state._y +=
        (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
    m_veh_state._psi += m_veh_state._wz * m_step;

    double new_time = t + m_step;
    // Write the output
    if (m_output) {
        if (std::abs(new_time - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(new_time);
            m_timeStepsStored++;
        }
    }

    return new_time;
}
// ======================================================================================================================
/// @brief Function call to integrate by just a single time step with jacobian computation. The order of the states in
// the jacobian matrix is as follows:
//  0: tiref_st._xe;
//  1: tiref_st._ye;
//  2: tirer_st._xe;
//  3: tirer_st._ye;
//  4: tiref_st._omega;
//  5: tirer_st._omega;
//  6: v_states._crankOmega; (only if torque converter is used)
//  7: v_states._x;
//  8: v_states._y;
//  9: v_states._u;
//  10: v_states._v;
//  11: v_states._psi;
//  12: v_states._wz;
// If the TMeasyNr tire is used then the order of the states in the jacbian matrix is as follows:
//  0: tiref_st._omega;
//  1: tirer_st._omega;
//  2: v_states._crankOmega; (only if torque converter is used)
//  3: v_states._x;
//  4: v_states._y;
//  5: v_states._u;
//  6: v_states._v;
//  7: v_states._psi;
//  8: v_states._wz;
/// @param t current time
/// @param throttle throttle input
/// @param steering steering input
/// @param braking braking input
/// @param on boolean to turn on jacobian computation
/// @return t + m_step
double d11SolverHalfImplicit::IntegrateStepWithJacobian(double t,
                                                        double throttle,
                                                        double steering,
                                                        double braking,
                                                        bool jacOn) {
    // Store header and first time step
    if (m_output && (t < m_step)) {
        Write(t);
        m_timeStepsStored++;
    }

    DriverInput controls(t, steering, throttle, braking);

    // If the jacobian switch is on, then compute the jacobian
    if (jacOn) {
        std::vector<double> y(m_num_states, 0);
        std::vector<double> ydot(m_num_states, 0);

        // package all the current states
        if (m_tire_type == TireType::TMeasy) {
            packY(m_veh_state, m_tireTMf_state, m_tireTMr_state, m_veh_param._tcbool, y);
        } else {
            packY(m_veh_state, m_tireTMNrf_state, m_tireTMNrr_state, m_veh_param._tcbool, y);
        }
        // ============================
        // Computing the state jacobian
        // ============================

        // Set a vector of del Ys - for now set this to some scale of y
        std::vector<double> delY(y.begin(), y.end());
        // In a loop perturb each state and get the corresponding perturbed ydot
        int ySize = y.size();

#pragma omp parallel for simd
        for (int i = 0; i < ySize; i++) {
            // Perturbation is 1e-3 * y (since some states are really small values wile some are huge)
            delY[i] = std::abs(delY[i] * 1e-3);
            if (delY[i] < 1e-8) {
                // This means that the particular state is 0. In this case set dels to 1e-3
                delY[i] = 1e-3;
            }

            // perturb y at i
            std::vector<double> perturbedYPlus(y.begin(), y.end());
            std::vector<double> perturbedYMinus(y.begin(), y.end());

            perturbedYPlus[i] = perturbedYPlus[i] + delY[i];
            perturbedYMinus[i] = perturbedYMinus[i] - delY[i];

            // ydots to store the output
            std::vector<double> ydotPlus(y.size(), 0.);
            std::vector<double> ydotMinus(y.size(), 0.);

            // Call the perturb function with these to get the perturbed ydot -> This does not update the state
            PerturbRhsFun(perturbedYPlus, controls, ydotPlus);
            PerturbRhsFun(perturbedYMinus, controls, ydotMinus);

// Update the jacobian matrix
#pragma omp simd
            for (int j = 0; j < ySize; j++) {
                m_jacobian_state[j][i] = (ydotPlus[j] - ydotMinus[j]) / (2 * delY[i]);
            }
        }

        // ==============================
        // Computing the control jacobian
        //===============================

        // Set a vector of del controls - for now we ingnore braking
        std::vector<double> delControls = {1e-3, 1e-3};
        // In a loop perturb each control and get the corresponding perturbed ydot
        int controlSize = delControls.size();

        for (int i = 0; i < controlSize; i++) {
            // perturb controls at i
            std::vector<double> perturbedControlsPlus = {steering, throttle};
            std::vector<double> perturbedControlsMinus = {steering, throttle};

            perturbedControlsPlus[i] = perturbedControlsPlus[i] + delControls[i];
            perturbedControlsMinus[i] = perturbedControlsMinus[i] - delControls[i];

            // ydots to store the output
            std::vector<double> ydotPlus(y.size(), 0.);
            std::vector<double> ydotMinus(y.size(), 0.);

            // Call the perturb function with these to get the perturbed ydot -> This does not update the state
            DriverInput inputPlus(t, perturbedControlsPlus[0], perturbedControlsPlus[1], 0);
            DriverInput inputMinus(t, perturbedControlsMinus[0], perturbedControlsMinus[1], 0);
            PerturbRhsFun(y, inputPlus, ydotPlus);
            PerturbRhsFun(y, inputMinus, ydotMinus);

            // Update the jacobian matrix
            for (int j = 0; j < ySize; j++) {
                m_jacobian_controls[j][i] = (ydotPlus[j] - ydotMinus[j]) / (2 * delControls[i]);
            }
        }
    }

    // Call the RHS function
    rhsFun(t, controls);

    // Integrate according to half implicit method for second order states
    // Integrate according to explicit method for first order states
    if (m_tire_type == TireType::TMeasy) {
        // First the tire states
        // F
        m_tireTMf_state._xe += m_tireTMf_state._xedot * m_step;
        m_tireTMf_state._ye += m_tireTMf_state._yedot * m_step;
        m_tireTMf_state._omega += m_tireTMf_state._dOmega * m_step;
        // R
        m_tireTMr_state._xe += m_tireTMr_state._xedot * m_step;
        m_tireTMr_state._ye += m_tireTMr_state._yedot * m_step;
        m_tireTMr_state._omega += m_tireTMr_state._dOmega * m_step;
    } else {
        // F
        m_tireTMNrf_state._omega += m_tireTMNrf_state._dOmega * m_step;
        // R
        m_tireTMNrr_state._omega += m_tireTMNrr_state._dOmega * m_step;
    }

    if (m_veh_param._tcbool) {
        m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
    }

    // Integrate velocity level first
    m_veh_state._u += m_veh_state._udot * m_step;
    m_veh_state._v += m_veh_state._vdot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // Integrate position level next
    m_veh_state._x +=
        (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
    m_veh_state._y +=
        (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
    m_veh_state._psi += m_veh_state._wz * m_step;

    double new_time = t + m_step;
    // Write the output
    if (m_output) {
        if (std::abs(new_time - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(new_time);
            m_timeStepsStored++;
        }
    }

    return new_time;
}

// ======================================================================================================================

/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations)
/// @param t Current time
void d11SolverHalfImplicit::rhsFun(double t) {
    // Get controls at the current timeStep
    auto controls = GetDriverInput(t, m_driver_data);

    // Calculate tire vertical loads
    std::vector<double> loads(2, 0);
    if (m_tire_type == TireType::TMeasy) {
        computeTireLoads(loads, m_veh_state, m_veh_param, m_tireTM_param);

        // Transform from vehicle frame to the tire frame
        vehToTireTransform(m_tireTMf_state, m_tireTMr_state, m_veh_state, loads, m_veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(m_tireTMf_state, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMr_state, m_tireTM_param, m_veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(m_veh_state, m_tireTMf_state, m_tireTMr_state, m_veh_param, m_tireTM_param, controls);
//////// DEBUG
#ifdef DEBUG
        M_DEBUG_F_TIRE_FX = m_tireTMf_state._fx;
        M_DEBUG_R_TIRE_FX = m_tireTMr_state._fx;

        M_DEBUG_F_TIRE_FY = m_tireTMf_state._fy;
        M_DEBUG_R_TIRE_FY = m_tireTMr_state._fy;

        M_DEBUG_F_TIRE_FZ = m_tireTMf_state._fz;
        M_DEBUG_R_TIRE_FZ = m_tireTMr_state._fz;
#endif

        // Vehicle dynamics
        tireToVehTransform(m_tireTMf_state, m_tireTMr_state, m_veh_state, m_veh_param, controls.m_steering);
        std::vector<double> fx = {m_tireTMf_state._fx, m_tireTMr_state._fx};
        std::vector<double> fy = {m_tireTMf_state._fy, m_tireTMr_state._fy};
        computeVehRHS(m_veh_state, m_veh_param, fx, fy);
    } else {
        computeTireLoads(loads, m_veh_state, m_veh_param, m_tireTMNr_param);

        // Transform from vehicle frame to the tire frame
        vehToTireTransform(m_tireTMNrf_state, m_tireTMNrr_state, m_veh_state, loads, m_veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(m_tireTMNrf_state, m_tireTMNr_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMNrr_state, m_tireTMNr_param, m_veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(m_veh_state, m_tireTMNrf_state, m_tireTMNrr_state, m_veh_param, m_tireTMNr_param,
                             controls);
//////// DEBUG
#ifdef DEBUG
        M_DEBUG_F_TIRE_FX = m_tireTMNrf_state._fx;
        M_DEBUG_R_TIRE_FX = m_tireTMNrr_state._fx;

        M_DEBUG_F_TIRE_FY = m_tireTMNrf_state._fy;
        M_DEBUG_R_TIRE_FY = m_tireTMNrr_state._fy;

        M_DEBUG_F_TIRE_FZ = m_tireTMNrf_state._fz;
        M_DEBUG_R_TIRE_FZ = m_tireTMNrr_state._fz;
#endif

        // Vehicle dynamics
        tireToVehTransform(m_tireTMNrf_state, m_tireTMNrr_state, m_veh_state, m_veh_param, controls.m_steering);
        std::vector<double> fx = {m_tireTMNrf_state._fx, m_tireTMNrr_state._fx};
        std::vector<double> fy = {m_tireTMNrf_state._fy, m_tireTMNrr_state._fy};
        computeVehRHS(m_veh_state, m_veh_param, fx, fy);
    }
}

// ======================================================================================================================

void d11SolverHalfImplicit::rhsFun(double t, DriverInput& controls) {
    // Calculate tire vertical loads
    std::vector<double> loads(2, 0);
    if (m_tire_type == TireType::TMeasy) {
        computeTireLoads(loads, m_veh_state, m_veh_param, m_tireTM_param);

        // Transform from vehicle frame to the tire frame
        vehToTireTransform(m_tireTMf_state, m_tireTMr_state, m_veh_state, loads, m_veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(m_tireTMf_state, m_tireTM_param, m_veh_param, controls.m_steering);

        computeTireRHS(m_tireTMr_state, m_tireTM_param, m_veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(m_veh_state, m_tireTMf_state, m_tireTMr_state, m_veh_param, m_tireTM_param, controls);

        // Vehicle dynamics
        tireToVehTransform(m_tireTMf_state, m_tireTMr_state, m_veh_state, m_veh_param, controls.m_steering);
        std::vector<double> fx = {m_tireTMf_state._fx, m_tireTMr_state._fx};
        std::vector<double> fy = {m_tireTMf_state._fy, m_tireTMr_state._fy};
        computeVehRHS(m_veh_state, m_veh_param, fx, fy);
    } else {
        computeTireLoads(loads, m_veh_state, m_veh_param, m_tireTMNr_param);

        // Transform from vehicle frame to the tire frame
        vehToTireTransform(m_tireTMNrf_state, m_tireTMNrr_state, m_veh_state, loads, m_veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(m_tireTMNrf_state, m_tireTMNr_param, m_veh_param, controls.m_steering);

        computeTireRHS(m_tireTMNrr_state, m_tireTMNr_param, m_veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(m_veh_state, m_tireTMNrf_state, m_tireTMNrr_state, m_veh_param, m_tireTMNr_param,
                             controls);

        // Vehicle dynamics
        tireToVehTransform(m_tireTMNrf_state, m_tireTMNrr_state, m_veh_state, m_veh_param, controls.m_steering);
        std::vector<double> fx = {m_tireTMNrf_state._fx, m_tireTMNrr_state._fx};
        std::vector<double> fy = {m_tireTMNrf_state._fy, m_tireTMNrr_state._fy};
        computeVehRHS(m_veh_state, m_veh_param, fx, fy);
    }
}

// ======================================================================================================================

// Function takes (y +- dely) and provides a new ydot for the pertubed y (ydot is the rhs of the system of equations)

void d11SolverHalfImplicit::PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot) {
    // Extract the vehicle and tire states vector state
    VehicleState veh_st;
    if (m_tire_type == TireType::TMeasy) {
        TMeasyState tiref_st;
        TMeasyState tirer_st;

        unpackY(y, m_veh_param._tcbool, veh_st, tiref_st, tirer_st);

        // Calculate tire vertical loads
        std::vector<double> loads(2, 0);
        computeTireLoads(loads, veh_st, m_veh_param, m_tireTM_param);

        // Transform from the vehicle frame to the tire frame
        vehToTireTransform(tiref_st, tirer_st, veh_st, loads, m_veh_param, controls.m_steering);

        // Tire dynamics
        computeTireRHS(tiref_st, m_tireTM_param, m_veh_param, controls.m_steering);

        computeTireRHS(tirer_st, m_tireTM_param, m_veh_param, 0);

        // Powertrain dynamics
        computePowertrainRHS(veh_st, tiref_st, tirer_st, m_veh_param, m_tireTM_param, controls);

        // Vehicle dynamics
        tireToVehTransform(tiref_st, tirer_st, veh_st, m_veh_param, controls.m_steering);
        std::vector<double> fx = {tiref_st._fx, tirer_st._fx};
        std::vector<double> fy = {tiref_st._fy, tirer_st._fy};
        computeVehRHS(veh_st, m_veh_param, fx, fy);

        // Pack the ydot and send it
        packYDOT(veh_st, tiref_st, tirer_st, m_veh_param._tcbool, ydot);
    } else {
        TMeasyNrState tiref_st;
        TMeasyNrState tirer_st;

        unpackY(y, m_veh_param._tcbool, veh_st, tiref_st, tirer_st);

        // Calculate tire vertical loads
        std::vector<double> loads(2, 0);
        computeTireLoads(loads, veh_st, m_veh_param, m_tireTMNr_param);

        // Transform from the vehicle frame to the tire frame
        vehToTireTransform(tiref_st, tirer_st, veh_st, loads, m_veh_param, controls.m_steering);

        // Tire dynamics
        computeTireRHS(tiref_st, m_tireTMNr_param, m_veh_param, controls.m_steering);

        computeTireRHS(tirer_st, m_tireTMNr_param, m_veh_param, 0);

        // Powertrain dynamics
        computePowertrainRHS(veh_st, tiref_st, tirer_st, m_veh_param, m_tireTMNr_param, controls);

        // Vehicle dynamics
        tireToVehTransform(tiref_st, tirer_st, veh_st, m_veh_param, controls.m_steering);
        std::vector<double> fx = {tiref_st._fx, tirer_st._fx};
        std::vector<double> fy = {tiref_st._fy, tirer_st._fy};
        computeVehRHS(veh_st, m_veh_param, fx, fy);

        // Pack the ydot and send it
        packYDOT(veh_st, tiref_st, tirer_st, m_veh_param._tcbool, ydot);
    }
}

void d11SolverHalfImplicit::Write(double t) {
    // If we are in initial time step, write the header
    if (t < m_step) {
        m_csv << "time";
        m_csv << "x";
        m_csv << "y";
        m_csv << "vx";
        m_csv << "vy";
        m_csv << "ax";
        m_csv << "ay";
        m_csv << "yaw";
        m_csv << "yaw_rate";
        m_csv << "wf";
        m_csv << "wr";
        m_csv << "sp_tor";
        m_csv << "current_gear";
        m_csv << "engine_omega";
#ifdef DEBUG
        m_csv << "f_tireForce_x";
        m_csv << "r_tireForce_x";
        m_csv << "f_tireForce_y";
        m_csv << "r_tireForce_y";
        m_csv << "f_tireForce_z";
        m_csv << "r_tireForce_z";
#endif
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
#ifdef DEBUG
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
        m_csv << 0;
#endif
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
    m_csv << m_veh_state._psi;
    m_csv << m_veh_state._wz;
    if (m_tire_type == TireType::TMeasy) {
        m_csv << m_tireTMf_state._omega;
        m_csv << m_tireTMr_state._omega;
    } else {
        m_csv << m_tireTMNrf_state._omega;
        m_csv << m_tireTMNrr_state._omega;
    }
    m_csv << m_veh_state._tor / 4.;
    m_csv << m_veh_state._current_gr + 1;
    m_csv << m_veh_state._crankOmega;
#ifdef DEBUG
    m_csv << M_DEBUG_F_TIRE_FX;
    m_csv << M_DEBUG_R_TIRE_FX;
    m_csv << M_DEBUG_F_TIRE_FY;
    m_csv << M_DEBUG_R_TIRE_FY;
    m_csv << M_DEBUG_F_TIRE_FZ;
    m_csv << M_DEBUG_R_TIRE_FZ;
#endif
    m_csv << std::endl;
}

// ======================================================================================================================

void d11SolverHalfImplicit::WriteToFile() {
    if (!m_output) {
        std::cout << "No output file specified. Call SetOutput() before calling WriteToFile()" << std::endl;
        return;
    }
    m_csv.write_to_file(m_output_file);
    m_csv.clearData();
    m_timeStepsStored = 0;
}

// ======================================================================================================================

// Utility functions for finite differencing

void packY(const d11::VehicleState& v_states,
           const d11::TMeasyState& tiref_st,
           const d11::TMeasyState& tirer_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Tire deflections (lf, rf, lr and rr)

    y[index++] = tiref_st._xe;
    y[index++] = tiref_st._ye;
    y[index++] = tirer_st._xe;
    y[index++] = tirer_st._ye;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tiref_st._omega;
    y[index++] = tirer_st._omega;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        y[index++] = v_states._crankOmega;
    }

    // Vehicle states
    y[index++] = v_states._x;    // X position
    y[index++] = v_states._y;    // Y position
    y[index++] = v_states._u;    // longitudinal velocity
    y[index++] = v_states._v;    // lateral velocity
    y[index++] = v_states._psi;  // yaw angle
    y[index++] = v_states._wz;   // yaw rate
}

void packY(const d11::VehicleState& v_states,
           const d11::TMeasyNrState& tiref_st,
           const d11::TMeasyNrState& tirer_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tiref_st._omega;
    y[index++] = tirer_st._omega;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        y[index++] = v_states._crankOmega;
    }

    // Vehicle states
    y[index++] = v_states._x;    // X position
    y[index++] = v_states._y;    // Y position
    y[index++] = v_states._u;    // longitudinal velocity
    y[index++] = v_states._v;    // lateral velocity
    y[index++] = v_states._psi;  // yaw angle
    y[index++] = v_states._wz;   // yaw rate
}

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyState& tiref_st,
              const d11::TMeasyState& tirer_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    ydot[index++] = tiref_st._xedot;
    ydot[index++] = tiref_st._yedot;
    ydot[index++] = tirer_st._xedot;
    ydot[index++] = tirer_st._yedot;

    ydot[index++] = tiref_st._dOmega;
    ydot[index++] = tirer_st._dOmega;

    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    ydot[index++] = v_states._dx;
    ydot[index++] = v_states._dy;
    ydot[index++] = v_states._udot;
    ydot[index++] = v_states._vdot;
    ydot[index++] = v_states._wz;
    ydot[index++] = v_states._wzdot;
}

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyNrState& tiref_st,
              const d11::TMeasyNrState& tirer_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    ydot[index++] = tiref_st._dOmega;
    ydot[index++] = tirer_st._dOmega;

    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    ydot[index++] = v_states._dx;
    ydot[index++] = v_states._dy;
    ydot[index++] = v_states._udot;
    ydot[index++] = v_states._vdot;
    ydot[index++] = v_states._wz;
    ydot[index++] = v_states._wzdot;
}

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyState& tiref_st,
             d11::TMeasyState& tirer_st) {
    int index = 0;
    // Tire deflections
    tiref_st._xe = y[index++];
    tiref_st._ye = y[index++];
    tirer_st._xe = y[index++];
    tirer_st._ye = y[index++];

    // Wheel angular velocities
    tiref_st._omega = y[index++];
    tirer_st._omega = y[index++];

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        v_states._crankOmega = y[index++];
    }

    // Vehicle states
    v_states._x = y[index++];    // X position
    v_states._y = y[index++];    // Y position
    v_states._u = y[index++];    // longitudinal velocity
    v_states._v = y[index++];    // lateral velocity
    v_states._psi = y[index++];  // yaw angle
    v_states._wz = y[index++];   // yaw rate
}
void unpackY(const std::vector<double>& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyNrState& tiref_st,
             d11::TMeasyNrState& tirer_st) {
    int index = 0;
    // Wheel angular velocities
    tiref_st._omega = y[index++];
    tirer_st._omega = y[index++];

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        v_states._crankOmega = y[index++];
    }

    // Vehicle states
    v_states._x = y[index++];    // X position
    v_states._y = y[index++];    // Y position
    v_states._u = y[index++];    // longitudinal velocity
    v_states._v = y[index++];    // lateral velocity
    v_states._psi = y[index++];  // yaw angle
    v_states._wz = y[index++];   // yaw rate
}