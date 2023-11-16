#ifndef DOF24_HALFIMPLICIT_H
#define DOF24_HALFIMPLICIT_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "dof24.h"

#ifdef USE_OPENMP
    #include <omp.h>
#endif

// =============================================================================
// Define the solver class
// =============================================================================
class d24SolverHalfImplicit {
  public:
    d24SolverHalfImplicit();
    ~d24SolverHalfImplicit();

    // Construct the solver with the specified vehicle
    // and tire parameters and the driver inputs.
    void Construct(const std::string& veh_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   const std::string& driver_file);
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   const std::string& driver_inputs_file,
                   TireType type);

    // Constructor for when a controller is used and we don't have a driver file
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file);
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   TireType type);

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
    void Initialize(d24::VehicleState& vehicle_states,
                    d24::TMeasyState& tire_states_LF,
                    d24::TMeasyState& tire_states_RF,
                    d24::TMeasyState& tire_states_LR,
                    d24::TMeasyState& tire_states_RR,
                    d24::SuspensionState& sus_states_LF,
                    d24::SuspensionState& sus_states_RF,
                    d24::SuspensionState& sus_states_LR,
                    d24::SuspensionState& sus_states_RR);

    void Initialize(d24::VehicleState& vehicle_states,
                    d24::TMeasyNrState& tire_states_LF,
                    d24::TMeasyNrState& tire_states_RF,
                    d24::TMeasyNrState& tire_states_LR,
                    d24::TMeasyNrState& tire_states_RR,
                    d24::SuspensionState& sus_states_LF,
                    d24::SuspensionState& sus_states_RF,
                    d24::SuspensionState& sus_states_LR,
                    d24::SuspensionState& sus_states_RR);

    double IntegrateStep(double t, double throttle, double steering, double braking);

    // Updates the Jacobian state and controls jacobian apart from integrating at each
    // In case you don't want it to update at each step, set the boolean to off
    double IntegrateStepWithJacobian(double t, double throttle, double steering, double braking, bool on);

    void WriteToFile();

    // Tire type getter
    TireType GetTireType() const { return m_tire_type; }

    // Vehicle states and tire states
    d24::VehicleState m_veh_state;
    d24::TMeasyState m_tireTMlf_state;
    d24::TMeasyState m_tireTMrf_state;
    d24::TMeasyState m_tireTMlr_state;
    d24::TMeasyState m_tireTMrr_state;
    d24::SuspensionState m_suslf_state;
    d24::SuspensionState m_susrf_state;
    d24::SuspensionState m_suslr_state;
    d24::SuspensionState m_susrr_state;
    d24::VehicleParam m_veh_param;     // vehicle parameters
    d24::TMeasyParam m_tireTM_param;   // Tire parameters
    d24::SuspensionParam m_sus_param;  // Suspension parameters

    // Tire states for the TM easy tire without relaxation
    d24::TMeasyNrState m_tireTMNrlf_state;
    d24::TMeasyNrState m_tireTMNrrf_state;
    d24::TMeasyNrState m_tireTMNrlr_state;
    d24::TMeasyNrState m_tireTMNrrr_state;
    d24::TMeasyNrParam m_tireTMNr_param;  // Tire parameters

  private:
    void Integrate();

    void Write(double t);

    void rhsFun(double t);

    void rhsFun(double t, DriverInput& controls);  // We need to provide controls when we are stepping

    // For finite differencing for applications in MPC to perturb either controls or y
    void PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot);

    // variable to store the tire type
    TireType m_tire_type;
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
#endif                  // OPENMP

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
#endif  // DEBUG
};

#ifndef SWIG
// Utility functions to help with finite differencing
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
           std::vector<double>& y);

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
           std::vector<double>& y);

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
              std::vector<double>& ydot);

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
              std::vector<double>& ydot);

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
             d24::SuspensionState& susrr_st);

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
             d24::SuspensionState& susrr_st);
#endif  // SWIG
#endif  // DOF24_HALFIMPLICIT_H