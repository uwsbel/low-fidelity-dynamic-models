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
void d24SolverHalfImplicit::Construct(const std::string& vehicle_params_file,
                                      const std::string& tire_params_file,
                                      const std::string& sus_params_file,
                                      const std::string& driver_inputs_file) {
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

    initializeTireSus(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                      m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_tireTM_param,
                      m_sus_param);

    // Size the jacobian matrices - size relies on the torque converter bool
    m_num_controls = 2;
    if (m_veh_param._tcbool) {
        m_num_states = 36;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    } else {
        m_num_states = 35;  // TODO: This is inaccurate
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
    initializeTireSus(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                      m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_tireTMNr_param,
                      m_sus_param);

    // Size the jacobian matrices - size relies on the torque converter bool
    m_num_controls = 2;
    if (m_veh_param._tcbool) {
        m_num_states = 28;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    } else {
        m_num_states = 27;  // TODO: This is inaccurate
        m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
        m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    }
}

// ======================================================================================================================
void d24SolverHalfImplicit::SetOutput(const std::string& output_file, double output_freq) {
    m_output = true;
    m_output_file = output_file;
    m_timeStepsStored = 0;
    m_dtout = 1.0 / output_freq;
}

// ======================================================================================================================
void d24SolverHalfImplicit::Solve() {
    assert(!m_driver_data.empty() && "No controls provided, please use construct to pass path to driver inputs");

    // For now just integrate to final time
    Integrate();
}
// ======================================================================================================================
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
            m_tireTMNrlf_state._xt += m_tireTMNrlf_state._dxt * m_step;
            m_tireTMNrlf_state._omega += m_tireTMNrlf_state._dOmega * m_step;
            // RF
            m_tireTMNrrf_state._xt += m_tireTMNrrf_state._dxt * m_step;
            m_tireTMNrrf_state._omega += m_tireTMNrrf_state._dOmega * m_step;
            // LR
            m_tireTMNrlr_state._xt += m_tireTMNrlr_state._dxt * m_step;
            m_tireTMNrlr_state._omega += m_tireTMNrlr_state._dOmega * m_step;
            // RR
            m_tireTMNrrr_state._xt += m_tireTMNrrr_state._dxt * m_step;
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
        m_veh_state._wx += m_veh_state._wxdot * m_step;
        m_veh_state._wy += m_veh_state._wydot * m_step;
        m_veh_state._wz += m_veh_state._wzdot * m_step;

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
double d24SolverHalfImplicit::IntegrateStep(double t, double throttle, double steering, double braking) {
    if (m_output && (t < m_step)) {
        Write(t);
        m_timeStepsStored++;
    }

    DriverInput controls(t, steering, throttle, braking);
    // Call the RHS function
    rhsFun(t, controls);

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
        m_tireTMNrlf_state._xt += m_tireTMNrlf_state._dxt * m_step;
        m_tireTMNrlf_state._omega += m_tireTMNrlf_state._dOmega * m_step;
        // RF
        m_tireTMNrrf_state._xt += m_tireTMNrrf_state._dxt * m_step;
        m_tireTMNrrf_state._omega += m_tireTMNrrf_state._dOmega * m_step;
        // LR
        m_tireTMNrlr_state._xt += m_tireTMNrlr_state._dxt * m_step;
        m_tireTMNrlr_state._omega += m_tireTMNrlr_state._dOmega * m_step;
        // RR
        m_tireTMNrrr_state._xt += m_tireTMNrrr_state._dxt * m_step;
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
    m_veh_state._wx += m_veh_state._wxdot * m_step;
    m_veh_state._wy += m_veh_state._wydot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // We have to recompute this here so that we are doing the half implicit
    // Now to integrate the cardan angles, we first write equations for them
    m_veh_state._dtheta = m_veh_state._wy * std::cos(m_veh_state._phi) - m_veh_state._wz * std::sin(m_veh_state._phi);
    m_veh_state._dpsi =
        (m_veh_state._wy * std::sin(m_veh_state._phi) / std::cos(m_veh_state._theta)) +
        (m_veh_state._wz * std::cos(m_veh_state._phi) / std::cos(m_veh_state._theta));  // Trouble when theta is 90?
    m_veh_state._dphi = m_veh_state._wx + m_veh_state._wy * std::sin(m_veh_state._phi) * std::tan(m_veh_state._theta) +
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
    double new_time = t + m_step;

    if (m_output) {
        if (std::abs(t - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(t);
            m_timeStepsStored++;
        }
    }

    return new_time;
}

//======================================================================================================================
double d24SolverHalfImplicit::IntegrateStepWithJacobian(double t,
                                                        double throttle,
                                                        double steering,
                                                        double braking,
                                                        bool jacOn) {
    if (m_output && (t < m_step)) {
        Write(t);
        m_timeStepsStored++;
    }
    DriverInput controls(t, steering, throttle, braking);

    if (jacOn) {
        std::vector<double> y(m_num_states, 0);
        std::vector<double> ydot(m_num_states, 0);

        if (m_tire_type == TireType::TMeasy) {
            packY(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state, m_suslf_state,
                  m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param._tcbool, y);
        } else {
            packY(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                  m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param._tcbool, y);
        }

        // ============================
        // Computing the state jacobian
        // ============================

        // Set a vector of del Ys - for now set this to some scale of y
        std::vector<double> delY(y.begin(), y.end());
        // In a loop pertub each state and get the corresponding perturbed ydot
        int ySize = y.size();

#pragma omp parallel for simd

        for (int i = 0; i < ySize; i++) {
            // Perterbation is 1e-3 * y (since some states are really small values wile some are huge)
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
        // In a loop pertub each control and get the corresponding perturbed ydot
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
        m_tireTMNrlf_state._xt += m_tireTMNrlf_state._dxt * m_step;
        m_tireTMNrlf_state._omega += m_tireTMNrlf_state._dOmega * m_step;
        // RF
        m_tireTMNrrf_state._xt += m_tireTMNrrf_state._dxt * m_step;
        m_tireTMNrrf_state._omega += m_tireTMNrrf_state._dOmega * m_step;
        // LR
        m_tireTMNrlr_state._xt += m_tireTMNrlr_state._dxt * m_step;
        m_tireTMNrlr_state._omega += m_tireTMNrlr_state._dOmega * m_step;
        // RR
        m_tireTMNrrr_state._xt += m_tireTMNrrr_state._dxt * m_step;
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
    m_veh_state._wx += m_veh_state._wxdot * m_step;
    m_veh_state._wy += m_veh_state._wydot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // We have to recompute this here so that we are doing the half implicit
    // Now to integrate the cardan angles, we first write equations for them
    m_veh_state._dtheta = m_veh_state._wy * std::cos(m_veh_state._phi) - m_veh_state._wz * std::sin(m_veh_state._phi);
    m_veh_state._dpsi =
        (m_veh_state._wy * std::sin(m_veh_state._phi) / std::cos(m_veh_state._theta)) +
        (m_veh_state._wz * std::cos(m_veh_state._phi) / std::cos(m_veh_state._theta));  // Trouble when theta is 90?
    m_veh_state._dphi = m_veh_state._wx + m_veh_state._wy * std::sin(m_veh_state._phi) * std::tan(m_veh_state._theta) +
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
    double new_time = t + m_step;

    if (m_output) {
        if (std::abs(t - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(t);
            m_timeStepsStored++;
        }
    }

    return new_time;
}
// ======================================================================================================================

void d24SolverHalfImplicit::rhsFun(double t) {
    // Get controls at the current timeStep
    auto controls = GetDriverInput(t, m_driver_data);

    if (m_tire_type == TireType::TMeasy) {
        vehToSusTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param);
        vehToTireTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                           m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param,
                           controls.m_steering);
        // Tire velocities
        computeTireRHS(m_veh_state, m_tireTMlf_state, m_veh_param, m_tireTM_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMrf_state, m_veh_param, m_tireTM_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMlr_state, m_veh_param, m_tireTM_param, 0);  // No rear steering
        computeTireRHS(m_veh_state, m_tireTMrr_state, m_veh_param, m_tireTM_param, 0);  // No rear steering
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
        computeSusRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
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
    } else {
        // TMeasy Nr states and structs need to be passed
        vehToSusTransform(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param);
        vehToTireTransform(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                           m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param,
                           controls.m_steering);
        // Tire velocities
        computeTireRHS(m_veh_state, m_tireTMNrlf_state, m_veh_param, m_tireTMNr_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMNrrf_state, m_veh_param, m_tireTMNr_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMNrlr_state, m_veh_param, m_tireTMNr_param, 0);  // No rear steering
        computeTireRHS(m_veh_state, m_tireTMNrrr_state, m_veh_param, m_tireTMNr_param, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                                       m_tireTMNrrr_state, m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state);
#ifdef DEBUG
        M_DEBUG_LF_TIRE_FX = m_tireTMNrlf_state._fx;
        M_DEBUG_RF_TIRE_FX = m_tireTMNrrf_state._fx;
        M_DEBUG_LR_TIRE_FX = m_tireTMNrlr_state._fx;
        M_DEBUG_RR_TIRE_FX = m_tireTMNrrr_state._fx;

        M_DEBUG_LF_TIRE_FY = m_tireTMNrlf_state._fy;
        M_DEBUG_RF_TIRE_FY = m_tireTMNrrf_state._fy;
        M_DEBUG_LR_TIRE_FY = m_tireTMNrlr_state._fy;
        M_DEBUG_RR_TIRE_FY = m_tireTMNrrr_state._fy;

        M_DEBUG_LF_TIRE_FZ = m_tireTMNrlf_state._fz;
        M_DEBUG_RF_TIRE_FZ = m_tireTMNrrf_state._fz;
        M_DEBUG_LR_TIRE_FZ = m_tireTMNrlr_state._fz;
        M_DEBUG_RR_TIRE_FZ = m_tireTMNrrr_state._fz;
#endif
        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                             m_tireTMNrrr_state, m_veh_param, m_tireTMNr_param, controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                           m_veh_param, controls.m_steering);
        // Suspension velocities
        computeSusRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                      m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_sus_param);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                                m_tireTMNrrr_state, m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state,
                                m_veh_param, m_tireTMNr_param, m_sus_param);

        // Compute the chassis accelerations
        computeVehicleRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_tireTMNr_param,
                          m_sus_param);
    }
}
// ======================================================================================================================

void d24SolverHalfImplicit::rhsFun(double t, DriverInput& controls) {
    if (m_tire_type == TireType::TMeasy) {
        vehToSusTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param);
        vehToTireTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                           m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param,
                           controls.m_steering);
        // Tire velocities
        computeTireRHS(m_veh_state, m_tireTMlf_state, m_veh_param, m_tireTM_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMrf_state, m_veh_param, m_tireTM_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMlr_state, m_veh_param, m_tireTM_param, 0);  // No rear steering
        computeTireRHS(m_veh_state, m_tireTMrr_state, m_veh_param, m_tireTM_param, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state,
                                       m_tireTMrr_state, m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state);

        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                             m_veh_param, m_tireTM_param, controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                           m_veh_param, controls.m_steering);
        // Suspension velocities
        computeSusRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
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
    } else {
        // TMeasy Nr states and structs need to be passed
        vehToSusTransform(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param);
        vehToTireTransform(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                           m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param,
                           controls.m_steering);
        // Tire velocities
        computeTireRHS(m_veh_state, m_tireTMNrlf_state, m_veh_param, m_tireTMNr_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMNrrf_state, m_veh_param, m_tireTMNr_param, controls.m_steering);
        computeTireRHS(m_veh_state, m_tireTMNrlr_state, m_veh_param, m_tireTMNr_param, 0);  // No rear steering
        computeTireRHS(m_veh_state, m_tireTMNrrr_state, m_veh_param, m_tireTMNr_param, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                                       m_tireTMNrrr_state, m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state);

        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                             m_tireTMNrrr_state, m_veh_param, m_tireTMNr_param, controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                           m_veh_param, controls.m_steering);
        // Suspension velocities
        computeSusRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                      m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_sus_param);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                                m_tireTMNrrr_state, m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state,
                                m_veh_param, m_tireTMNr_param, m_sus_param);

        // Compute the chassis accelerations
        computeVehicleRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                          m_suslf_state, m_susrf_state, m_suslr_state, m_susrr_state, m_veh_param, m_tireTMNr_param,
                          m_sus_param);
    }
}
// ======================================================================================================================
// Function takes (y +- dely) and provides a new ydot for the pertubed y (ydot is the rhs of the system of equations)
void d24SolverHalfImplicit::PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot) {
    VehicleState veh_st;
    SuspensionState suslf_st;
    SuspensionState susrf_st;
    SuspensionState suslr_st;
    SuspensionState susrr_st;
    if (m_tire_type == TireType::TMeasy) {
        TMeasyState tirelf_st;
        TMeasyState tirerf_st;
        TMeasyState tirelr_st;
        TMeasyState tirerr_st;

        unpackY(y, m_veh_param._tcbool, veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st,
                suslr_st, susrr_st);

        vehToSusTransform(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                          m_veh_param);
        vehToTireTransform(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                           m_veh_param, controls.m_steering);
        // Tire velocities
        computeTireRHS(veh_st, tirelf_st, m_veh_param, m_tireTM_param, controls.m_steering);
        computeTireRHS(veh_st, tirerf_st, m_veh_param, m_tireTM_param, controls.m_steering);
        computeTireRHS(veh_st, tirelr_st, m_veh_param, m_tireTM_param, 0);  // No rear steering
        computeTireRHS(veh_st, tirerr_st, m_veh_param, m_tireTM_param, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st,
                                       susrr_st);

        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, m_tireTM_param, controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, controls.m_steering);
        // Suspension velocities
        computeSusRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                      m_veh_param, m_sus_param);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st,
                                susrr_st, m_veh_param, m_tireTM_param, m_sus_param);

        // Compute the chassis accelerations
        computeVehicleRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                          m_veh_param, m_tireTM_param, m_sus_param);

        packYDOT(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                 m_veh_param._tcbool, ydot);

    } else {
        TMeasyNrState tirelf_st;
        TMeasyNrState tirerf_st;
        TMeasyNrState tirelr_st;
        TMeasyNrState tirerr_st;
        // TMeasy Nr states and structs need to be passed
        vehToSusTransform(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                          m_veh_param);
        vehToTireTransform(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                           m_veh_param, controls.m_steering);
        // Tire velocities
        computeTireRHS(veh_st, tirelf_st, m_veh_param, m_tireTMNr_param, controls.m_steering);
        computeTireRHS(veh_st, tirerf_st, m_veh_param, m_tireTMNr_param, controls.m_steering);
        computeTireRHS(veh_st, tirelr_st, m_veh_param, m_tireTMNr_param, 0);  // No rear steering
        computeTireRHS(veh_st, tirerr_st, m_veh_param, m_tireTMNr_param, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st,
                                       susrr_st);

        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, m_tireTMNr_param,
                             controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, controls.m_steering);
        // Suspension velocities
        computeSusRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                      m_veh_param, m_sus_param);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st,
                                susrr_st, m_veh_param, m_tireTMNr_param, m_sus_param);

        // Compute the chassis accelerations
        computeVehicleRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                          m_veh_param, m_tireTMNr_param, m_sus_param);
        packYDOT(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, suslf_st, susrf_st, suslr_st, susrr_st,
                 m_veh_param._tcbool, ydot);
    }
}
// ======================================================================================================================
void d24SolverHalfImplicit::Write(double t) {
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
        m_csv << "pitch";
        m_csv << "roll_rate";
        m_csv << "yaw_rate";
        m_csv << "pitch_rate";
        m_csv << "wlf";
        m_csv << "wrf";
        m_csv << "wlr";
        m_csv << "wrr";
        m_csv << "sp_tor";
        m_csv << "current_gear";
        m_csv << "engine_omega";
#ifdef DEBUG
        m_csv << "lf_tireForce_x";
        m_csv << "rf_tireForce_x";
        m_csv << "lr_tireForce_x";
        m_csv << "rr_tireForce_x";
        m_csv << "lf_tireForce_y";
        m_csv << "rf_tireForce_y";
        m_csv << "lr_tireForce_y";
        m_csv << "rr_tireForce_y";
        m_csv << "lf_tireForce_z";
        m_csv << "rf_tireForce_z";
        m_csv << "lr_tireForce_z";
        m_csv << "rr_tireForce_z";
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
    m_csv << m_veh_state._phi;
    m_csv << m_veh_state._psi;
    m_csv << m_veh_state._theta;
    m_csv << m_veh_state._wx;
    m_csv << m_veh_state._wz;
    m_csv << m_veh_state._wy;
    if (m_tire_type == TireType::TMeasy) {
        m_csv << m_tireTMlf_state._omega;
        m_csv << m_tireTMrf_state._omega;
        m_csv << m_tireTMlr_state._omega;
        m_csv << m_tireTMrr_state._omega;
    } else {
        m_csv << m_tireTMNrlf_state._omega;
        m_csv << m_tireTMNrrf_state._omega;
        m_csv << m_tireTMNrlr_state._omega;
        m_csv << m_tireTMNrrr_state._omega;
    }
    m_csv << m_veh_state._tor / 4.;
    m_csv << m_veh_state._current_gr + 1;
    m_csv << m_veh_state._crankOmega;
#ifdef DEBUG
    m_csv << M_DEBUG_LF_TIRE_FX;
    m_csv << M_DEBUG_RF_TIRE_FX;
    m_csv << M_DEBUG_LR_TIRE_FX;
    m_csv << M_DEBUG_RR_TIRE_FX;
    m_csv << M_DEBUG_LF_TIRE_FY;
    m_csv << M_DEBUG_RF_TIRE_FY;
    m_csv << M_DEBUG_LR_TIRE_FY;
    m_csv << M_DEBUG_RR_TIRE_FY;
    m_csv << M_DEBUG_LF_TIRE_FZ;
    m_csv << M_DEBUG_RF_TIRE_FZ;
    m_csv << M_DEBUG_LR_TIRE_FZ;
    m_csv << M_DEBUG_RR_TIRE_FZ;
#endif
    m_csv << std::endl;
}

// ======================================================================================================================

void d24SolverHalfImplicit::WriteToFile() {
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
void packY(const d24::VehicleState& v_states,
           const d24::TMeasyState& tirelf_st,
           const d24::TMeasyState& tirerf_st,
           const d24::TMeasyState& tirelr_st,
           const d24::TMeasyState& tirerr_st,
           const d24::SuspensionState& suslf_st,
           const d24::SuspensionState& susrf_st,
           const d24::SuspensionState& suslr_st,
           const d24::SuspensionState& susrr_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Tire deflections in all three directions
    y[index++] = tirelf_st._xe;
    y[index++] = tirelf_st._ye;
    y[index++] = tirelf_st._xt;
    y[index++] = tirerf_st._xe;
    y[index++] = tirerf_st._ye;
    y[index++] = tirerf_st._xt;
    y[index++] = tirelr_st._xe;
    y[index++] = tirelr_st._ye;
    y[index++] = tirelr_st._xt;
    y[index++] = tirerr_st._xe;
    y[index++] = tirerr_st._ye;
    y[index++] = tirerr_st._xt;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tirelf_st._omega;
    y[index++] = tirerf_st._omega;
    y[index++] = tirelr_st._omega;
    y[index++] = tirerr_st._omega;

    // Suspension states
    y[index++] = suslf_st._wu;
    y[index++] = suslf_st._xs;
    y[index++] = susrf_st._wu;
    y[index++] = susrf_st._xs;
    y[index++] = suslr_st._wu;
    y[index++] = suslr_st._xs;
    y[index++] = susrr_st._wu;
    y[index++] = susrr_st._xs;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        y[index++] = v_states._crankOmega;
    }

    // Vehicle states
    y[index++] = v_states._x;  // X position
    y[index++] = v_states._y;  // Y position
    y[index++] = v_states._u;  // longitudinal velocity
    y[index++] = v_states._v;  // lateral velocity
    y[index++] = v_states._w;  // Vertical velocity

    // Angular velocities
    y[index++] = v_states._wz;  // yaw rate
    y[index++] = v_states._wx;  // roll rate
    y[index++] = v_states._wy;  // pitch rate

    // Cardan Angles
    y[index++] = v_states._psi;    // yaw angle
    y[index++] = v_states._phi;    // roll angle
    y[index++] = v_states._theta;  // pitch angle
}

void packY(const d24::VehicleState& v_states,
           const d24::TMeasyNrState& tirelf_st,
           const d24::TMeasyNrState& tirerf_st,
           const d24::TMeasyNrState& tirelr_st,
           const d24::TMeasyNrState& tirerr_st,
           const d24::SuspensionState& suslf_st,
           const d24::SuspensionState& susrf_st,
           const d24::SuspensionState& suslr_st,
           const d24::SuspensionState& susrr_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Tire deflections only in vertical direction
    y[index++] = tirelf_st._xt;
    y[index++] = tirerf_st._xt;
    y[index++] = tirelr_st._xt;
    y[index++] = tirerr_st._xt;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tirelf_st._omega;
    y[index++] = tirerf_st._omega;
    y[index++] = tirelr_st._omega;
    y[index++] = tirerr_st._omega;

    // Suspension states
    y[index++] = suslf_st._wu;
    y[index++] = suslf_st._xs;
    y[index++] = susrf_st._wu;
    y[index++] = susrf_st._xs;
    y[index++] = suslr_st._wu;
    y[index++] = suslr_st._xs;
    y[index++] = susrr_st._wu;
    y[index++] = susrr_st._xs;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        y[index++] = v_states._crankOmega;
    }

    // Vehicle states
    y[index++] = v_states._x;  // X position
    y[index++] = v_states._y;  // Y position
    y[index++] = v_states._u;  // longitudinal velocity
    y[index++] = v_states._v;  // lateral velocity
    y[index++] = v_states._w;  // Vertical velocity

    // Angular velocities
    y[index++] = v_states._wz;  // yaw rate
    y[index++] = v_states._wx;  // roll rate
    y[index++] = v_states._wy;  // pitch rate

    // Cardan Angles
    y[index++] = v_states._psi;    // yaw angle
    y[index++] = v_states._phi;    // roll angle
    y[index++] = v_states._theta;  // pitch angle
}

void packYDOT(const d24::VehicleState& v_states,
              const d24::TMeasyState& tirelf_st,
              const d24::TMeasyState& tirerf_st,
              const d24::TMeasyState& tirelr_st,
              const d24::TMeasyState& tirerr_st,
              const d24::SuspensionState& suslf_st,
              const d24::SuspensionState& susrf_st,
              const d24::SuspensionState& suslr_st,
              const d24::SuspensionState& susrr_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    // Tire deflections in all three directions
    ydot[index++] = tirelf_st._xedot;
    ydot[index++] = tirelf_st._yedot;
    ydot[index++] = tirelf_st._dxt;
    ydot[index++] = tirerf_st._xedot;
    ydot[index++] = tirerf_st._yedot;
    ydot[index++] = tirerf_st._dxt;
    ydot[index++] = tirelr_st._xedot;
    ydot[index++] = tirelr_st._yedot;
    ydot[index++] = tirelr_st._dxt;
    ydot[index++] = tirerr_st._xedot;
    ydot[index++] = tirerr_st._yedot;
    ydot[index++] = tirerr_st._dxt;

    // Wheel angular velocities (lf, rf, lr and rr)
    ydot[index++] = tirelf_st._dOmega;
    ydot[index++] = tirerf_st._dOmega;
    ydot[index++] = tirelr_st._dOmega;
    ydot[index++] = tirerr_st._dOmega;

    // Suspension states
    ydot[index++] = suslf_st._dwu;
    ydot[index++] = suslf_st._dxs;
    ydot[index++] = susrf_st._dwu;
    ydot[index++] = susrf_st._dxs;
    ydot[index++] = suslr_st._dwu;
    ydot[index++] = suslr_st._dxs;
    ydot[index++] = susrr_st._dwu;
    ydot[index++] = susrr_st._dxs;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    // Vehicle states
    ydot[index++] = v_states._dx;    // X position
    ydot[index++] = v_states._dy;    // Y position
    ydot[index++] = v_states._udot;  // longitudinal velocity
    ydot[index++] = v_states._vdot;  // lateral velocity
    ydot[index++] = v_states._wdot;  // Vertical velocity

    // Angular velocities
    ydot[index++] = v_states._wzdot;  // yaw rate
    ydot[index++] = v_states._wxdot;  // roll rate
    ydot[index++] = v_states._wydot;  // pitch rate

    // Cardan Angles
    ydot[index++] = v_states._dpsi;    // yaw angle
    ydot[index++] = v_states._dphi;    // roll angle
    ydot[index++] = v_states._dtheta;  // pitch angle
}

void packYDOT(const d24::VehicleState& v_states,
              const d24::TMeasyNrState& tirelf_st,
              const d24::TMeasyNrState& tirerf_st,
              const d24::TMeasyNrState& tirelr_st,
              const d24::TMeasyNrState& tirerr_st,
              const d24::SuspensionState& suslf_st,
              const d24::SuspensionState& susrf_st,
              const d24::SuspensionState& suslr_st,
              const d24::SuspensionState& susrr_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    // Tire deflections in all three directions
    ydot[index++] = tirelf_st._dxt;
    ydot[index++] = tirerf_st._dxt;
    ydot[index++] = tirelr_st._dxt;
    ydot[index++] = tirerr_st._dxt;

    // Wheel angular velocities (lf, rf, lr and rr)
    ydot[index++] = tirelf_st._dOmega;
    ydot[index++] = tirerf_st._dOmega;
    ydot[index++] = tirelr_st._dOmega;
    ydot[index++] = tirerr_st._dOmega;

    // Suspension states
    ydot[index++] = suslf_st._dwu;
    ydot[index++] = suslf_st._dxs;
    ydot[index++] = susrf_st._dwu;
    ydot[index++] = susrf_st._dxs;
    ydot[index++] = suslr_st._dwu;
    ydot[index++] = suslr_st._dxs;
    ydot[index++] = susrr_st._dwu;
    ydot[index++] = susrr_st._dxs;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    // Vehicle states
    ydot[index++] = v_states._dx;    // X position
    ydot[index++] = v_states._dy;    // Y position
    ydot[index++] = v_states._udot;  // longitudinal velocity
    ydot[index++] = v_states._vdot;  // lateral velocity
    ydot[index++] = v_states._wdot;  // Vertical velocity

    // Angular velocities
    ydot[index++] = v_states._wzdot;  // yaw rate
    ydot[index++] = v_states._wxdot;  // roll rate
    ydot[index++] = v_states._wydot;  // pitch rate

    // Cardan Angles
    ydot[index++] = v_states._dpsi;    // yaw angle
    ydot[index++] = v_states._dphi;    // roll angle
    ydot[index++] = v_states._dtheta;  // pitch angle
}

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d24::VehicleState& v_states,
             d24::TMeasyState& tirelf_st,
             d24::TMeasyState& tirerf_st,
             d24::TMeasyState& tirelr_st,
             d24::TMeasyState& tirerr_st,
             d24::SuspensionState& suslf_st,
             d24::SuspensionState& susrf_st,
             d24::SuspensionState& suslr_st,
             d24::SuspensionState& susrr_st) {
    int index = 0;
    // Tire deflections in all three directions
    tirelf_st._xe = y[index++];
    tirelf_st._ye = y[index++];
    tirelf_st._xt = y[index++];
    tirerf_st._xe = y[index++];
    tirerf_st._ye = y[index++];
    tirerf_st._xt = y[index++];
    tirelr_st._xe = y[index++];
    tirelr_st._ye = y[index++];
    tirelr_st._xt = y[index++];
    tirerr_st._xe = y[index++];
    tirerr_st._ye = y[index++];
    tirerr_st._xt = y[index++];

    // Wheel angular velocities (lf, rf, lr, and rr)
    tirelf_st._omega = y[index++];
    tirerf_st._omega = y[index++];
    tirelr_st._omega = y[index++];
    tirerr_st._omega = y[index++];

    // Suspension states
    suslf_st._wu = y[index++];
    suslf_st._xs = y[index++];
    susrf_st._wu = y[index++];
    susrf_st._xs = y[index++];
    suslr_st._wu = y[index++];
    suslr_st._xs = y[index++];
    susrr_st._wu = y[index++];
    susrr_st._xs = y[index++];

    // Crank angular velocity - This is a state only when a torque converter is used
    if (has_TC) {
        v_states._crankOmega = y[index++];
    }

    // Vehicle states
    v_states._x = y[index++];  // X position
    v_states._y = y[index++];  // Y position
    v_states._u = y[index++];  // longitudinal velocity
    v_states._v = y[index++];  // lateral velocity
    v_states._w = y[index++];  // Vertical velocity

    // Angular velocities
    v_states._wz = y[index++];  // yaw rate
    v_states._wx = y[index++];  // roll rate
    v_states._wy = y[index++];  // pitch rate

    // Cardan Angles
    v_states._psi = y[index++];    // yaw angle
    v_states._phi = y[index++];    // roll angle
    v_states._theta = y[index++];  // pitch angle
}

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d24::VehicleState& v_states,
             d24::TMeasyNrState& tirelf_st,
             d24::TMeasyNrState& tirerf_st,
             d24::TMeasyNrState& tirelr_st,
             d24::TMeasyNrState& tirerr_st,
             d24::SuspensionState& suslf_st,
             d24::SuspensionState& susrf_st,
             d24::SuspensionState& suslr_st,
             d24::SuspensionState& susrr_st) {
    int index = 0;
    // Tire deflections in all three directions
    tirelf_st._xt = y[index++];
    tirerf_st._xt = y[index++];
    tirelr_st._xt = y[index++];
    tirerr_st._xt = y[index++];

    // Wheel angular velocities (lf, rf, lr, and rr)
    tirelf_st._omega = y[index++];
    tirerf_st._omega = y[index++];
    tirelr_st._omega = y[index++];
    tirerr_st._omega = y[index++];

    // Suspension states
    suslf_st._wu = y[index++];
    suslf_st._xs = y[index++];
    susrf_st._wu = y[index++];
    susrf_st._xs = y[index++];
    suslr_st._wu = y[index++];
    suslr_st._xs = y[index++];
    susrr_st._wu = y[index++];
    susrr_st._xs = y[index++];

    // Crank angular velocity - This is a state only when a torque converter is used
    if (has_TC) {
        v_states._crankOmega = y[index++];
    }

    // Vehicle states
    v_states._x = y[index++];  // X position
    v_states._y = y[index++];  // Y position
    v_states._u = y[index++];  // longitudinal velocity
    v_states._v = y[index++];  // lateral velocity
    v_states._w = y[index++];  // Vertical velocity

    // Angular velocities
    v_states._wz = y[index++];  // yaw rate
    v_states._wx = y[index++];  // roll rate
    v_states._wy = y[index++];  // pitch rate

    // Cardan Angles
    v_states._psi = y[index++];    // yaw angle
    v_states._phi = y[index++];    // roll angle
    v_states._theta = y[index++];  // pitch angle
}