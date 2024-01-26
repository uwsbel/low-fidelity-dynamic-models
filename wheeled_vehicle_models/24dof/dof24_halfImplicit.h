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
// Solver class
// =============================================================================
class d24SolverHalfImplicit {
  public:
    d24SolverHalfImplicit();
    ~d24SolverHalfImplicit();

    /// @brief Construct the solver using path to vehicle parameters, tire parameters, suspension parameters and driver
    /// inputs. The tire type defaults to TMEasy
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param sus_params_file Path to the suspension parameter json file
    /// @param driver_inputs_file Path to the driver inputs text file
    void Construct(const std::string& veh_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   const std::string& driver_file);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters,suspension parameters, driver
    /// inputs and the TireType (Either TMEasy or TMEasyNr).
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param sus_params_file Path to the suspension parameter json file
    /// @param driver_inputs_file Path to the driver inputs text file
    /// @param type TireType (Either TMEasy or TMEasyNr)
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   const std::string& driver_inputs_file,
                   d24::TireType type);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, suspension parameters.
    /// TireType defualts to TMEasy tires

    /// This function signature is mainly provided for cases where the driver inputs are not available at the start of
    /// the simulation but rather come from a controller during the simualtion.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param sus_params_file Path to the suspension parameter json file
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, suspension parameters and the
    /// TireType (Either TMEasy or TMEasyNr).

    ///  This is mainly provided for cases where the driver inputs are not available at the start of the
    /// simulation but rather come from a controller during the simualtion and the user wants to specify a TireType.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param sus_params_file Path to the suspension parameter json file
    /// @param type TireType (Either TMEasy or TMEasyNr)
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   d24::TireType type);

    /// @brief Set the simulation time step used to integrate all the vehicles using the solver
    /// @param step time step to set
    void SetTimeStep(double step) { m_step = step; }

    /// @brief Getter function for the simulation end time. This is calculated from the driver inputs if provided
    double GetEndT() { return m_tend; }

    /// @brief Getter function for simulation time step
    double GetStep() { return m_step; }

    /// @brief Get the DriverData struct which contains the driver inputs set during solver construction
    DriverData GetDriverData() { return m_driver_data; }

    /// @brief Get the "State Jacobian"

    /// The "State Jacobian" is defined as the derivative of the system RHS with respect to the state vector at a
    /// particular time step. The order of elements in jacobian can be seen in the implementation of the packY function.
    /// See demos/HMMWV/demo_hmmwv_hi_stepWithJac.cpp for example
    /// @return State Jacobian matrix (in the form of vector of double vectors)
    std::vector<std::vector<double>> GetJacobianState() { return m_jacobian_state; }

    /// @brief Get the "Control Jacobian"
    /// @return The "Control Jacobian" is defined as the derivative of the system RHS with respect to the control vector
    /// (steering, throttle in this order) at a particular time step. See demos/HMMWV/demo_hmmwv_hi_stepWithJac.cpp for
    /// example
    /// @return Control Jacobian matrix (in the form of vector of double vectors)
    std::vector<std::vector<double>> GetJacobianControls() { return m_jacobian_controls; }

    /// @brief Switch on output to a csv file at the specified frequency.

    /// @param output_file Path to the output file
    /// @param output_freq Frequency of data output
    void SetOutput(const std::string& output_file, double output_freq);

    /// @brief Solve the system of equations and run the simulation uptil m_tend

    /// Note: If the user wants to simulate step by step (where step is the simualtion time step), use IntegrateStep or
    /// IntegrateStepWithJacobian (in case m_jacobian_state and m_jacobian_controls is desired). If the user has set the
    /// output to a csv file, then the csv files will be written to the data/output folder.
    void Solve();

    /// @brief Initialize vehicle, tire and suspension states.

    /// This function has to be called before solve and after the
    ///  Construct function is called. Although it is possible to provide non-zero intial states, this is untested and
    ///  it is recommended to use the default zero states. For examples of use, see demos.
    ///  @param vehicle_states Vehicle states
    ///  @param tire_states_LF Left Front (LF) TMeasy tire states
    ///  @param tire_states_RF Right Front (RF) TMeasy tire states
    ///  @param tire_states_LR Left Rear (LR) TMeasy tire states
    ///  @param tire_states_RR Right Rear (RR) TMeasy tire states
    ///  @param sus_states_LF Left Front (LF) suspension states
    ///  @param sus_states_RF Right Front (RF) suspension states
    ///  @param sus_states_LR Left Rear (LR) suspension states
    ///  @param sus_states_RR Right Rear (RR) suspension states
    void Initialize(d24::VehicleState& vehicle_states,
                    d24::TMeasyState& tire_states_LF,
                    d24::TMeasyState& tire_states_RF,
                    d24::TMeasyState& tire_states_LR,
                    d24::TMeasyState& tire_states_RR,
                    d24::SuspensionState& sus_states_LF,
                    d24::SuspensionState& sus_states_RF,
                    d24::SuspensionState& sus_states_LR,
                    d24::SuspensionState& sus_states_RR);

    /// @brief Initialize vehicle, tire and suspension states. Overloaded for TMesayNr tires.

    /// This function has to be called before solve and after the
    /// construct. Although it is possible to provide non-zero intial states, this is untested and it is recommended to
    /// use the default zero states.
    /// @param vehicle_states Vehicle states
    /// @param tire_states_LF Left Front (LF) TMeasyNr tire states
    /// @param tire_states_RF Right Front (RF) TMeasyNr tire states
    /// @param tire_states_LR Left Rear (LR) TMeasyNr tire states
    /// @param tire_states_RR Right Rear (RR) TMeasyNr tire states
    /// @param sus_states_LF Left Front (LF) suspension states
    /// @param sus_states_RF Right Front (RF) suspension states
    /// @param sus_states_LR Left Rear (LR) suspension states
    /// @param sus_states_RR Right Rear (RR) suspension states
    void Initialize(d24::VehicleState& vehicle_states,
                    d24::TMeasyNrState& tire_states_LF,
                    d24::TMeasyNrState& tire_states_RF,
                    d24::TMeasyNrState& tire_states_LR,
                    d24::TMeasyNrState& tire_states_RR,
                    d24::SuspensionState& sus_states_LF,
                    d24::SuspensionState& sus_states_RF,
                    d24::SuspensionState& sus_states_LR,
                    d24::SuspensionState& sus_states_RR);
    /// @brief This function can be used when the user wants to integrate the system step by step based on the set
    /// time-step of the solver

    /// The user can call SetTimeStep to set this time step.
    /// @param t Current time
    /// @param throttle Throttle input
    /// @param steering Steering input
    /// @param braking Braking input
    /// @return The time at the end of the integration step
    double IntegrateStep(double t, double throttle, double steering, double braking);

    /// @brief This function can be used when the user not only wants to integrate the system step by step based on the
    /// time-step of the solver but also wants to get the jacobian of the system at the end of the integration step.

    /// The user can call SetTimeStep to set this time step. The jacobian is calculated using finite differencing with
    /// openMP (if enabled while building). For more details on what Jacobian is being calculated, see the documentation
    /// for GetJacobianState and GetJacobianControls.
    /// @param t Current time
    /// @param throttle Throttle input
    /// @param steering Steering input
    /// @param braking Braking input
    /// @param on Boolean that indicates whether the Jacobian is to be calculated for this integration step or not. This
    /// is useful when the user wants to integrate the system step by step but only wants the Jacobian at certain time
    /// steps.
    /// @return The time at the end of the integration step
    double IntegrateStepWithJacobian(double t, double throttle, double steering, double braking, bool on);

    /// @brief Get the tire type being used by the solver
    /// @return TireType (Either TMEasy or TMEasyNr)
    d24::TireType GetTireType() const { return m_tire_type; }

    d24::VehicleState m_veh_state;  //!< Vehicle state - can be accessed any time after construction
    d24::TMeasyState
        m_tireTMlf_state;  //!< TMeasy Tire state (Left Front (LF)) - can be accessed any time after construction
    d24::TMeasyState
        m_tireTMrf_state;  //!< TMeasy Tire state (Right Front (RF)) - can be accessed any time after construction
    d24::TMeasyState
        m_tireTMlr_state;  //!< TMeasy Tire state (Left Rear (LR)) - can be accessed any time after construction
    d24::TMeasyState
        m_tireTMrr_state;  //!< TMeasy Tire state (Right Rear (RR)) - can be accessed any time after construction
    d24::SuspensionState
        m_suslf_state;  //!< Suspension state (Left Front (LF)) - can be accessed any time after construction
    d24::SuspensionState
        m_susrf_state;  //!< Suspension state (Right Front (RF)) - can be accessed any time after construction
    d24::SuspensionState
        m_suslr_state;  //!< Suspension state (Left Rear (LR)) - can be accessed any time after construction
    d24::SuspensionState
        m_susrr_state;  //!< Suspension state (Right Rear (RR)) - can be accessed any time after construction
    d24::VehicleParam m_veh_param;     //!< Vehicle parameters - can be accessed any time after construction
    d24::TMeasyParam m_tireTM_param;   //!< TMeasy tire parameters - can be accessed any time after construction
    d24::SuspensionParam m_sus_param;  //!< Suspension parameters - can be accessed any time after construction

    // Tire states for the TM easy tire without relaxation
    d24::TMeasyNrState
        m_tireTMNrlf_state;  //!< TMeasyNr Tire state (Left Front (LF)) - can be accessed any time after construction
    d24::TMeasyNrState
        m_tireTMNrrf_state;  //!< TMeasyNr Tire state (Right Front (RF)) - can be accessed any time after construction
    d24::TMeasyNrState
        m_tireTMNrlr_state;  //!< TMeasyNr Tire state (Left Rear (LR)) - can be accessed any time after construction
    d24::TMeasyNrState
        m_tireTMNrrr_state;  //!< TMeasyNr Tire state (Right Rear (RR)) - can be accessed any time after construction
    d24::TMeasyNrParam m_tireTMNr_param;  //!< TMeasyNr tire parameters - can be accessed any time after construction

    /// @brief Write csv_writer to csv file in case of step by step simulation

    /// In case the user calls IntegrateStepWithJacobian or IntegrateStep as well as set's output to true, then
    /// at each step states are only written to the csv writer object not to the csv file itself. Thus the user needs to
    /// call this function to write the data to the csv file by calling WriteToFile.
    void WriteToFile();

  private:
    void Integrate();

    void Write(double t);

    void rhsFun(double t);

    void rhsFun(double t, DriverInput& controls);  // We need to provide controls when we are stepping

    // For finite differencing for applications in MPC to perturb either controls or y
    void PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot);

    // variable to store the tire type
    d24::TireType m_tire_type;
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
/// @brief Utility functions to help with finite differencing

/// All simulation states are packed into a single vector y
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
/// @brief Utility functions to help with finite differencing. Overloaded for TMeasyNr tire

/// All simulation states are packed into a single vector y.
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

/// @brief Utility functions to help with finite differencing.

/// All simulation RHS accelerations and velocities are packed into a single vector ydot
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

/// @brief Utility functions to help with finite differencing. Overloaded for TMeasyNr tire

/// All simulation RHS accelerations and velocities are packed into a single vector ydot
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

/// @brief Utility functions to help with finite differencing.

/// Unpacks y back to the vehicle, tire and suspension state structs
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
/// @brief Utility functions to help with finite differencing. Overloaded for TMeasyNr tire

/// Unpacks y back to the vehicle, tire and suspension state structs
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