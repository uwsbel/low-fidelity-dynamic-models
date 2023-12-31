#ifndef DOF11_HALFIMPLICIT_H
#define DOF11_HALFIMPLICIT_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "dof11.h"

#ifdef USE_OPENMP
    #include <omp.h>
#endif

// =============================================================================
// Define the solver class
// =============================================================================

class d11SolverHalfImplicit {
  public:
    d11SolverHalfImplicit();
    ~d11SolverHalfImplicit();

    // Construct the solver with the specified vehicle
    // and tire parameters and the driver inputs.
    void Construct(const std::string& veh_params_file,
                   const std::string& tire_params_file,
                   const std::string& driver_file);
    void Construct(const std::string& veh_params_file,
                   const std::string& tire_params_file,
                   const std::string& driver_file,
                   TireType type);

    // Constructor for when a controller is used and we don't have a driver file
    void Construct(const std::string& vehicle_params_file, const std::string& tire_params_file);
    void Construct(const std::string& vehicle_params_file, const std::string& tire_params_file, TireType type);

    // Set the solver time step
    void SetTimeStep(double step) { m_step = step; }

    double GetEndT() { return m_tend; }

    double GetStep() { return m_step; }

    DriverData GetDriverData() { return m_driver_data; }

    // Jacobian getter functions
    std::vector<std::vector<double>> GetJacobianState() { return m_jacobian_state; }
    std::vector<std::vector<double>> GetJacobianControls() { return m_jacobian_controls; }

    // Set the output file path
    void SetOutput(const std::string& output_file, double output_freq);

    // Solve uptill the end time specified in the driver inputs
    void Solve();

    // Initialize function in case user wants non-zero intializations
    void Initialize(d11::VehicleState& vehicle_states,
                    d11::TMeasyState& tire_states_F,
                    d11::TMeasyState& tire_states_R);
    void Initialize(d11::VehicleState& vehicle_states,
                    d11::TMeasyNrState& tire_states_F,
                    d11::TMeasyNrState& tire_states_R);

    double IntegrateStep(double t, double throttle, double steering, double braking);

    // Updates the Jacobian state and controls jacobian apart from integrating at each
    // In case you don't want it to update at each step, set the boolean to off
    double IntegrateStepWithJacobian(double t, double throttle, double steering, double braking, bool on);

    void WriteToFile();

    // Vehicle states and tire states
    d11::VehicleState m_veh_state;
    d11::TMeasyState m_tireTMf_state;
    d11::TMeasyState m_tireTMr_state;
    d11::VehicleParam m_veh_param;    // vehicle parameters
    d11::TMeasyParam m_tireTM_param;  // Tire parameters

    // TMeasyNr tire states
    d11::TMeasyNrState m_tireTMNrf_state;
    d11::TMeasyNrState m_tireTMNrr_state;
    d11::TMeasyNrParam m_tireTMNr_param;

  private:
    void Integrate();

    void Write(double t);

    void rhsFun(double t);

    void rhsFun(double t, DriverInput& controls);  // We need to provide controls when we are stepping

    // For finite differencing for applications in MPC to perturb either controls or y
    void PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot);

    TireType m_tire_type;       // Tire type
    CSV_writer m_csv;           // CSV writer object
    double m_tend;              // final integration time
    double m_step;              // integration time step
    int m_timeStepsStored;      // Keeps track of time steps stored if we need data output
    bool m_output;              // data output flag
    double m_dtout;             // time interval between data output
    std::string m_output_file;  // output file path
    DriverData m_driver_data;   // driver inputs
    int m_num_controls;         // Number of control states for jacobian
    int m_num_states;           // Number of states for jacobian

    // Jacobian matrix incase user needs finite differencing
    std::vector<std::vector<double>> m_jacobian_state;
    std::vector<std::vector<double>> m_jacobian_controls;

#ifdef USE_OPENMP
    int m_num_threads;  // number of threads
#endif

    ////////// DEBUG
#ifdef DEBUG
    double M_DEBUG_F_TIRE_FX;
    double M_DEBUG_R_TIRE_FX;

    double M_DEBUG_F_TIRE_FY;
    double M_DEBUG_R_TIRE_FY;

    double M_DEBUG_F_TIRE_FZ;
    double M_DEBUG_R_TIRE_FZ;
#endif
};

#ifndef SWIG
// Utility functions to help with finite differencing
void packY(const d11::VehicleState& v_states,
           const d11::TMeasyState& tiref_st,
           const d11::TMeasyState& tirer_st,
           bool has_TC,
           std::vector<double>& y);

void packY(const d11::VehicleState& v_states,
           const d11::TMeasyNrState& tiref_st,
           const d11::TMeasyNrState& tirer_st,
           bool has_TC,
           std::vector<double>& y);

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyState& tiref_st,
              const d11::TMeasyState& tirer_st,
              bool has_TC,
              std::vector<double>& ydot);

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyNrState& tiref_st,
              const d11::TMeasyNrState& tirer_st,
              bool has_TC,
              std::vector<double>& ydot);

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyState& tiref_st,
             d11::TMeasyState& tirer_st);

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyNrState& tiref_st,
             d11::TMeasyNrState& tirer_st);

#endif

#endif