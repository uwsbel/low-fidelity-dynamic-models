#ifndef DOF18_HALFIMPLICIT_H
#define DOF18_HALFIMPLICIT_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "dof18.h"

#ifdef USE_OPENMP
    #include <omp.h>
#endif

// =============================================================================
// Define the solver class
// =============================================================================

class d18SolverHalfImplicit {
  public:
    d18SolverHalfImplicit();
    ~d18SolverHalfImplicit();

    // Construct the solver with the specified vehicle
    // and tire parameters and the driver inputs.
    void Construct(const std::string& veh_params_file,
                   const std::string& tire_params_file,
                   const std::string& driver_file);

    // Constructor for when a controller is used and we don't have a driver file
    void Construct(const std::string& vehicle_params_file, const std::string& tire_params_file);

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
    void Initialize(d18::VehicleState& vehicle_states,
                    d18::TMeasyState& tire_states_LF,
                    d18::TMeasyState& tire_states_RF,
                    d18::TMeasyState& tire_states_LR,
                    d18::TMeasyState& tire_states_RR);

    double IntegrateStep(double t, double throttle, double steering, double braking);

    // Updates the Jacobian state and controls jacobian apart from integrating at each
    // In case you don't want it to update at each step, set the boolean to off
    double IntegrateStepWithJacobian(double t, double throttle, double steering, double braking, bool on);

    void WriteToFile();

    // Vehicle states and tire states
    d18::VehicleState m_veh_state;
    d18::TMeasyState m_tirelf_state;
    d18::TMeasyState m_tirerf_state;
    d18::TMeasyState m_tirelr_state;
    d18::TMeasyState m_tirerr_state;
    d18::VehicleParam m_veh_param;  // vehicle parameters
    d18::TMeasyParam m_tire_param;  // Tire parameters

  private:
    void Integrate();

    void Write(double t);

    void rhsFun(double t);

    void rhsFun(double t, DriverInput& controls);  // We need to provide controls when we are stepping

    // For finite differencing for applications in MPC to perturb either controls or y
    void PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot);

    CSV_writer m_csv;           // CSV writer object
    double m_tend;              // final integration time
    double m_step;              // integration time step
    int m_timeStepsStored;      // Keeps track of time steps stored if we need data output
    bool m_output;              // data output flag
    double m_dtout;             // time interval between data output
    std::string m_output_file;  // output file path
    DriverData m_driver_data;   // driver inputs

    // Jacobian matrix incase user needs finite differencing
    std::vector<std::vector<double>> m_jacobian_state;
    std::vector<std::vector<double>> m_jacobian_controls;

#ifdef USE_OPENMP
    int m_num_threads;  // number of threads
#endif

    ////////// DEBUG
#ifdef DEBUG
    double M_DEBUG_LF_TIRE_FX;
    double M_DEBUG_RF_TIRE_FX;
    double M_DEBUG_LR_TIRE_FX;
    double M_DEBUG_RR_TIRE_FX;

    double M_DEBUG_LF_TIRE_FY;
    double M_DEBUG_RF_TIRE_FY;
    double M_DEBUG_LR_TIRE_FY;
    double M_DEBUG_RR_TIRE_FY;

    double M_DEBUG_LF_TIRE_FZ;
    double M_DEBUG_RF_TIRE_FZ;
    double M_DEBUG_LR_TIRE_FZ;
    double M_DEBUG_RR_TIRE_FZ;
#endif
};

#ifndef SWIG
// Utility functions to help with finite differencing
void packY(const d18::VehicleState& v_states,
           const d18::TMeasyState& tirelf_st,
           const d18::TMeasyState& tirerf_st,
           const d18::TMeasyState& tirelr_st,
           const d18::TMeasyState& tirerr_st,
           bool has_TC,
           std::vector<double>& y);

void packYDOT(const d18::VehicleState& v_states,
              const d18::TMeasyState& tirelf_st,
              const d18::TMeasyState& tirerf_st,
              const d18::TMeasyState& tirelr_st,
              const d18::TMeasyState& tirerr_st,
              bool has_TC,
              std::vector<double>& ydot);

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d18::VehicleState& v_states,
             d18::TMeasyState& tirelf_st,
             d18::TMeasyState& tirerf_st,
             d18::TMeasyState& tirelr_st,
             d18::TMeasyState& tirerr_st);

#endif

#endif