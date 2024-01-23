#ifndef DOF18_SUNDIALS_H
#define DOF18_SUNDIALS_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "dof24.h"

// Sundials includes
#include <cvodes/cvodes.h>
#include <nvector/nvector_serial.h>
#include <sundials/sundials_types.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sunmatrix/sunmatrix_dense.h>
#include <sunnonlinsol/sunnonlinsol_fixedpoint.h>

// Don't need any of this in the python wrapper
#ifndef SWIG
    #define Ith(v, i) NV_Ith_S(v, i)

    #define ZERO RCONST(0.0)
    #define ONE RCONST(1.00)

    #if defined(SUNDIALS_EXTENDED_PRECISION)
        #define GSYM "Lg"
        #define ESYM "Le"
        #define FSYM "Lf"
    #else
        #define GSYM "g"
        #define ESYM "e"
        #define FSYM "f"
    #endif

// =============================================================================

/// @brief The Sundials solver supports Forward Sensitivity Analysis (FSA) for the Engine and Steering Map parameters.
/// These can be set using the ParamFlag enum.
namespace ParamFlag {
enum Enum {
    NONE = 0,               //
    ENGINE_MAP = 1 << 0,    //
    STEERING_MAP = 1 << 1,  //
    ALL = 0xFFFF            //
};
}
// Forward declaration required for friend class
class d24SolverSundials;
namespace d24 {
/// @brief User Data structure to pass to the RHS function for the sundials solver. Handled internally

/// The RHS functions that the sundials solver uses to integrate the vehicle model take very specific arguments. In
/// order to pass paramateres, driver input and reference paths (in case FSA is enabled) to the RHS functions, we need
/// to pack them into a single structure. This is done in the UserData class. This is not exposed to the user, and is
/// handled internally by the solver.
class UserData {
  public:
    UserData() : m_current_gr(0), m_param_flags(0), m_vx_idx(0), m_vy_idx(0) {}
    UserData(int param_flags,
             const d24::VehicleParam& veh_param,
             const d24::TMeasyParam& tire_TM,
             const d24::SuspensionParam& sus_param,
             const DriverData& driver_data,
             const PathData& ref_path = PathData());
    UserData(int param_flags,
             const d24::VehicleParam& veh_param,
             const d24::TMeasyNrParam& tire_TMNr,
             const d24::SuspensionParam& sus_param,
             const DriverData& driver_data,
             const PathData& ref_path = PathData());

    std::vector<realtype>& GetParams() { return m_params; }
    std::vector<realtype>& GetParamScales() { return m_param_scales; }

    const d24::VehicleParam& GetVehicleParam();
    const d24::TMeasyParam& GetTireTMParam();
    const d24::TMeasyNrParam& GetTireTMNrParam();
    const d24::SuspensionParam& GetSuspensionParam();
    const DriverData& GetDriverData() const { return m_driver_data; }
    const PathData& GetReferencePath() const { return m_ref_path; }
    bool HasReferencePath() const { return !m_ref_path.empty(); }

    void SetCurrentGear(int gear) { m_current_gr = gear; }
    int GetCurrentGear() const { return m_current_gr; }
    TireType GetTireType() const { return m_tire_type; }

    // Indices of vehicle position in state vectors
    int m_vx_idx;
    int m_vy_idx;

  private:
    bool IsFlagSet(ParamFlag::Enum val) const { return (m_param_flags & static_cast<int>(val)) != 0; }

    d24::VehicleParam m_veh_param;
    d24::TMeasyParam m_tireTM_param;
    d24::TMeasyNrParam m_tireTMNr_param;
    d24::SuspensionParam m_sus_param;
    DriverData m_driver_data;
    PathData m_ref_path;

    // Current transmission gear
    int m_current_gr;
    // Tire type - changes how we pack and unpack the state vectors
    TireType m_tire_type;

    // Sensitivity parameters
    int m_param_flags;
    std::vector<realtype> m_params;
    std::vector<realtype> m_param_scales;

    friend class ::d24SolverSundials;
};

// =============================================================================

/// @brief Objective class for FSA. Handled internally
struct Objective {
    bool success;
    realtype value;
    std::vector<realtype> gradient;
};
}  // namespace d24
#endif
class d24SolverSundials {
  public:
    d24SolverSundials();
    ~d24SolverSundials();

    /// @brief Construct the solver using path to vehicle parameters, tire parameters, suspension parameters and driver
    /// inputs. The tire type defaults to TMEasy
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param sus_params_file Path to the suspension parameter json file
    /// @param driver_inputs_file Path to the driver inputs text file
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& sus_params_file,
                   const std::string& driver_inputs_file);
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
                   TireType type);

    /// @brief Set the absolute and relative tolerances for the solver.

    /// Since the sundials integrator is a error-controlling integrator, users need to specify the accepted relative and
    /// absolute error tolerances. See sundials documentation for more details -
    /// https://sundials.readthedocs.io/en/latest/cvodes/Mathematics_link.html
    /// @param rtol Relative tolerance
    /// @param atol Absolute tolerance
    void SetTolerances(double rtol, double atol);

    /// @brief Set the maximum step size for the solver.

    /// Since the sundials integrator is an adaptive time step integrator, users can and only need to specify the
    /// maximum time-step that the solver can take to integrata the system. See sundials documentation for more details
    /// - https://sundials.readthedocs.io/en/latest/cvodes/Mathematics_link.html
    /// @param hmax Maximum time-step (s)
    void SetMaxStep(double hmax);

    /// @brief In case Forward Sensitivity Analysis (FSA) is enabled, this function sets the parameters for which
    /// sensitivities are desired.
    /// @param param_flags Can be either ParamFlag::ENGINE_MAP, ParamFlag::STEERING_MAP or ParamFlag::ALL
    /// @param params The values of the EngineMap or the SteeringMap or both depending on the param_flags
    void SetSensitivityParameters(int param_flags, const std::vector<double>& params);

    /// @brief Overload to make it sundials compatible. Can also be used by user
    /// @param param_flags Can be either ParamFlag::ENGINE_MAP, ParamFlag::STEERING_MAP or ParamFlag::ALL
    /// @param params The values of the EngineMap or the SteeringMap or both depending on the param_flags
    void SetSensitivityParameters(int param_flags, const double* params);

    void SetSensitivityParameterScales(const std::vector<double>& scales);
    void SetSensitivityParameterScales(const double* scales);

    /// @brief Set the reference path for Foward Sensitivity Analysis (FSA).

    /// Currently FSA is only supported with point wise loss on the reference path. The reference path is a text file
    /// with the ground truth vehicle positions (x,y) at a frequency of 10 Hz.
    /// @param reference_path_file Path to the reference path text file
    void SetReferencePath(const std::string& reference_path_file);

    /// @brief Set the solver to verbose mode (prints out solver statistics)
    /// @param verbose Boolean flag to enable/disable verbose mode
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// @brief Switch on output to a csv file at a frequency of 10 Hz.

    /// @param output_file Path to the output file
    void SetOutput(const std::string& output_file);

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
    int Initialize(d24::VehicleState& vehicle_states,
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
    int Initialize(d24::VehicleState& vehicle_states,
                   d24::TMeasyNrState& tire_states_LF,
                   d24::TMeasyNrState& tire_states_RF,
                   d24::TMeasyNrState& tire_states_LR,
                   d24::TMeasyNrState& tire_states_RR,
                   d24::SuspensionState& sus_states_LF,
                   d24::SuspensionState& sus_states_RF,
                   d24::SuspensionState& sus_states_LR,
                   d24::SuspensionState& sus_states_RR);

    /// @brief Get the number of equations to be integrated. Used internally
    /// @return number of equations
    int GetNumStates() const { return m_neq; }

    /// @brief Get the number of parameters in FSA. Used internally
    /// @return number of parameters
    int GetNumParameters() const { return m_ns; }

    /// @brief Get the TireType
    /// @return TireType (Either TMEasy or TMEasyNr)
    TireType GetTireType() const { return m_tire_type; }

    /// @brief Solve the system of equations and run the simulation uptil m_tend

    /// Since the system equations or all ODE's with initial values, we use CVODES from sundials to solve the system. To
    /// better understand how sundials solves the system of equations, see the sundials documentation -
    /// https://sundials.readthedocs.io/en/latest/cvodes/Mathematics_link.html
    /// @param fsa Boolean flag to enable/disable FSA
    bool Solve(bool fsa);

    /// @brief This function should be called if the user wants evaluate the value and the gradient of the cost function
    /// in addition to solving the system

    /// Note: Reference path needs to be set else an assertion is raised
    /// @param gradient Whether the gradient is required or not
    /// @return Objective which holds the value (Objective.value) and the gradient (Objective.gradient[]) with respect
    /// to the parameters
    d24::Objective Evaluate_FSA(bool gradient);

  private:
    bool Integrate(bool fsa);

    void Write(CSV_writer& csv, realtype t, N_Vector y, N_Vector* yS);

    bool m_has_TC;
    int m_offset;

    int m_method;          // CVODES integration method
    int m_mode;            // CVODES return mode
    d24::UserData m_data;  // CVODES user data

    realtype m_tend;  // final integration time
    realtype m_hmax;  // maximum step size

    realtype m_rtol;                // relative tolerance
    realtype m_atol;                // absolute tolerance
    std::vector<realtype> m_atolS;  // sensitivity absolute tolerances

    SUNContext m_sunctx;  // Sundials context
    void* m_cvode_mem;    // CVODES solver memory

    int m_neq;        // number of equations (states)
    int m_ns;         // number of sensitivities (parameters)
    N_Vector m_y0;    // initial state conditions
    N_Vector m_q0;    // initial quadrature conditions
    N_Vector* m_yS0;  // initial state sensitivity conditions
    N_Vector* m_qS0;  // initial quadrature sensitivity conditions

    bool m_verbose;             // verbose output?
    bool m_output;              // generate output file?
    std::string m_output_file;  // output file name
    TireType m_tire_type;       // Tire type - changes how we pack and unpack the state vectors
};
#ifndef SWIG
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
           N_Vector& y);

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
           N_Vector& y);

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
              N_Vector& ydot);

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
              N_Vector& ydot);

void unpackY(const N_Vector& y,
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

void unpackY(const N_Vector& y,
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
namespace d24 {
void calcF();
void calcFtL();
void calcJtL();
// Function to evaluate f(t, y, ydot) to be used by Sundials ODE solver
int rhsFun(realtype t, N_Vector y, N_Vector ydot, void* user_data);

int rhsQuad(realtype t, N_Vector y, N_Vector qdot, void* user_data);

int rhsQuadSens(int Ns,
                realtype t,
                N_Vector y,
                N_Vector* yS,
                N_Vector qdot,
                N_Vector* qSdot,
                void* user_data,
                N_Vector tmp,
                N_Vector tmpQ);

int printStatsCvode(void* cvode_mem);

static int check_retval(void* returnvalue, const char* funcname, int opt) {
    int* retval;

    /* Check if SUNDIALS function returned NULL pointer - no memory allocated */
    if (opt == 0 && returnvalue == NULL) {
        fprintf(stderr, "\nSUNDIALS_ERROR: %s() failed - returned NULL pointer\n\n", funcname);
        return (1);
    }

    /* Check if retval < 0 */
    else if (opt == 1) {
        retval = (int*)returnvalue;
        if (*retval < 0) {
            fprintf(stderr, "\nSUNDIALS_ERROR: %s() failed with retval = %d\n\n", funcname, *retval);
            return (1);
        }
    }

    /* Check if function returned NULL pointer - no memory allocated */
    else if (opt == 2 && returnvalue == NULL) {
        fprintf(stderr, "\nMEMORY_ERROR: %s() failed - returned NULL pointer\n\n", funcname);
        return (1);
    }

    return (0);
}

void printSUNMatrix(SUNMatrix A, sunindextype matrows, sunindextype matcols);
}  // namespace d24
#endif
#endif