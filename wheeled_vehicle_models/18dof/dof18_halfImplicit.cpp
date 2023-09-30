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
d18SolverHalfImplicit::d18SolverHalfImplicit() : m_tend(0), m_step(0.001), m_output(false), m_csv(" ") {
#ifdef USE_OPENMP
    m_num_threads = omp_get_num_procs() / 4;
    // m_num_threads = 6;
    omp_set_num_threads(m_num_threads);
#endif
}
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

// Overload for situations when a controller is used and we don't have a driver data file
void d18SolverHalfImplicit::Construct(const std::string& vehicle_params_file, const std::string& tire_params_file) {
    // Load vehicle and tire parameters
    setVehParamsJSON(m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_tire_param, tire_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(m_tire_param);
}
// ======================================================================================================================

/// @brief Initialize vehicle and tire states in case something other than non zero is needed - has to be called after
/// construct for now
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

    // Size the jacobian matrices - size relies on the torque converter bool
    if (m_veh_param._tcbool) {
        m_jacobian_state.resize(21, std::vector<double>(21, 0));
        m_jacobian_controls.resize(21, std::vector<double>(2, 0));
    } else {
        m_jacobian_state.resize(20, std::vector<double>(20, 0));
        m_jacobian_controls.resize(20, std::vector<double>(2, 0));
    }
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
    assert(!m_driver_data.empty() && "No controls provided, please use construct to pass path to driver inputs");

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
/// @param throttle throttle input
/// @param steering steering input
/// @param braking braking input
/// @return t + m_step
double d18SolverHalfImplicit::IntegrateStep(double t, double throttle, double steering, double braking) {
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
//  0: tirelf_st._xe;
//  1: tirelf_st._ye;
//  2: tirerf_st._xe;
//  3: tirerf_st._ye;
//  4: tirelr_st._xe;
//  5: tirelr_st._ye;
//  6: tirerr_st._xe;
//  7: tirerr_st._ye;
//  8: tirelf_st._omega;
//  9: tirerf_st._omega;
//  10: tirelr_st._omega;
//  11: tirerr_st._omega;
//  12: v_states._crankOmega; (only if torque converter is used)
//  13: v_states._x;
//  14: v_states._y;
//  15: v_states._u;
//  16: v_states._v;
//  17: v_states._psi;
//  18: v_states._wz;
//  19: v_states._phi;
//  20: v_states._wx;
/// @param t current time
/// @param throttle throttle input
/// @param steering steering input
/// @param braking braking input
/// @param on boolean to turn on jacobian computation
/// @return t + m_step
double d18SolverHalfImplicit::IntegrateStepWithJacobian(double t,
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
        std::vector<double> y(20, 0);
        std::vector<double> ydot(20, 0);
        if (m_veh_param._tcbool) {
            y.resize(21);
            ydot.resize(21);
        }

        // package all the current states
        packY(m_veh_state, m_tirelf_state, m_tirerf_state, m_tirelr_state, m_tirerr_state, m_veh_param._tcbool, y);

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
//////// DEBUG
#ifdef DEBUG
    M_DEBUG_LF_TIRE_FX = m_tirelf_state._fx;
    M_DEBUG_RF_TIRE_FX = m_tirerf_state._fx;
    M_DEBUG_LR_TIRE_FX = m_tirelr_state._fx;
    M_DEBUG_RR_TIRE_FX = m_tirerr_state._fx;

    M_DEBUG_LF_TIRE_FY = m_tirelf_state._fy;
    M_DEBUG_RF_TIRE_FY = m_tirerf_state._fy;
    M_DEBUG_LR_TIRE_FY = m_tirelr_state._fy;
    M_DEBUG_RR_TIRE_FY = m_tirerr_state._fy;

    M_DEBUG_LF_TIRE_FZ = m_tirelf_state._fz;
    M_DEBUG_RF_TIRE_FZ = m_tirerf_state._fz;
    M_DEBUG_LR_TIRE_FZ = m_tirelr_state._fz;
    M_DEBUG_RR_TIRE_FZ = m_tirerr_state._fz;
#endif

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

// Function takes (y +- dely) and provides a new ydot for the pertubed y (ydot is the rhs of the system of equations)

void d18SolverHalfImplicit::PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot) {
    // Extract the vehicle and tire states vector state
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    unpackY(y, m_veh_param._tcbool, veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    // Calculate tire vertical loads
    std::vector<double> loads(4, 0);
    computeTireLoads(loads, veh_st, m_veh_param, m_tire_param);

    // Transform from the vehicle frame to the tire frame
    vehToTireTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh_st, loads, m_veh_param, controls.m_steering);

    // Tire dynamics
    computeTireRHS(tirelf_st, m_tire_param, m_veh_param, controls.m_steering);
    computeTireRHS(tirerf_st, m_tire_param, m_veh_param, controls.m_steering);
    computeTireRHS(tirelr_st, m_tire_param, m_veh_param, 0);
    computeTireRHS(tirerr_st, m_tire_param, m_veh_param, 0);

    // Powertrain dynamics
    computePowertrainRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, m_tire_param, controls);

    // Vehicle dynamics
    tireToVehTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh_st, m_veh_param, controls.m_steering);
    std::vector<double> fx = {tirelf_st._fx, tirerf_st._fx, tirelr_st._fx, tirerr_st._fx};
    std::vector<double> fy = {tirelf_st._fy, tirerf_st._fy, tirelr_st._fy, tirerr_st._fy};
    computeVehRHS(veh_st, m_veh_param, fx, fy);

    // Pack the ydot and send it
    packYDOT(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param._tcbool, ydot);
}

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
    m_csv << m_veh_state._wx;
    m_csv << m_veh_state._wz;
    m_csv << m_tirelf_state._omega;
    m_csv << m_tirerf_state._omega;
    m_csv << m_tirelr_state._omega;
    m_csv << m_tirerr_state._omega;
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

void d18SolverHalfImplicit::WriteToFile() {
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

void packY(const d18::VehicleState& v_states,
           const d18::TMeasyState& tirelf_st,
           const d18::TMeasyState& tirerf_st,
           const d18::TMeasyState& tirelr_st,
           const d18::TMeasyState& tirerr_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Tire deflections (lf, rf, lr and rr)

    y[index++] = tirelf_st._xe;
    y[index++] = tirelf_st._ye;
    y[index++] = tirerf_st._xe;
    y[index++] = tirerf_st._ye;
    y[index++] = tirelr_st._xe;
    y[index++] = tirelr_st._ye;
    y[index++] = tirerr_st._xe;
    y[index++] = tirerr_st._ye;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tirelf_st._omega;
    y[index++] = tirerf_st._omega;
    y[index++] = tirelr_st._omega;
    y[index++] = tirerr_st._omega;

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
    y[index++] = v_states._phi;  // roll angle
    y[index++] = v_states._wx;   // roll rate
}

void packYDOT(const d18::VehicleState& v_states,
              const d18::TMeasyState& tirelf_st,
              const d18::TMeasyState& tirerf_st,
              const d18::TMeasyState& tirelr_st,
              const d18::TMeasyState& tirerr_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    ydot[index++] = tirelf_st._xedot;
    ydot[index++] = tirelf_st._yedot;
    ydot[index++] = tirerf_st._xedot;
    ydot[index++] = tirerf_st._yedot;
    ydot[index++] = tirelr_st._xedot;
    ydot[index++] = tirelr_st._yedot;
    ydot[index++] = tirerr_st._xedot;
    ydot[index++] = tirerr_st._yedot;

    ydot[index++] = tirelf_st._dOmega;
    ydot[index++] = tirerf_st._dOmega;
    ydot[index++] = tirelr_st._dOmega;
    ydot[index++] = tirerr_st._dOmega;

    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    ydot[index++] = v_states._dx;
    ydot[index++] = v_states._dy;
    ydot[index++] = v_states._udot;
    ydot[index++] = v_states._vdot;
    ydot[index++] = v_states._wz;
    ydot[index++] = v_states._wzdot;
    ydot[index++] = v_states._wx;
    ydot[index++] = v_states._wxdot;
}

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d18::VehicleState& v_states,
             d18::TMeasyState& tirelf_st,
             d18::TMeasyState& tirerf_st,
             d18::TMeasyState& tirelr_st,
             d18::TMeasyState& tirerr_st) {
    int index = 0;
    // Tire deflections
    tirelf_st._xe = y[index++];
    tirelf_st._ye = y[index++];
    tirerf_st._xe = y[index++];
    tirerf_st._ye = y[index++];
    tirelr_st._xe = y[index++];
    tirelr_st._ye = y[index++];
    tirerr_st._xe = y[index++];
    tirerr_st._ye = y[index++];

    // Wheel angular velocities
    tirelf_st._omega = y[index++];
    tirerf_st._omega = y[index++];
    tirelr_st._omega = y[index++];
    tirerr_st._omega = y[index++];

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
    v_states._phi = y[index++];  // roll angle
    v_states._wx = y[index++];   // roll rate
}