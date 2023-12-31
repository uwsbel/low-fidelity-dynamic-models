#ifndef DOF18_SUNDIALS_H
#define DOF18_SUNDIALS_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "dof11.h"

// Sundials includes
#include <cvodes/cvodes.h>
#include <nvector/nvector_serial.h>
#include <sundials/sundials_types.h>
#include <sunlinsol/sunlinsol_dense.h>
#include <sunmatrix/sunmatrix_dense.h>
#include <sunnonlinsol/sunnonlinsol_fixedpoint.h>
// Some sundials defines to make life easier

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

// Flags to specify SA parameters in UserData
namespace ParamFlag {
enum Enum {
    NONE = 0,               //
    ENGINE_MAP = 1 << 0,    //
    STEERING_MAP = 1 << 1,  //
    ALL = 0xFFFF            //
};
}

// Struct for user data passed through Sundials solvers
class UserData {
  public:
    UserData() : m_current_gr(0), m_param_flags(0), m_vx_idx(0), m_vy_idx(0) {}
    UserData(int param_flags,
             const d11::VehicleParam& veh_param,
             const d11::TMeasyParam& tire,
             const DriverData& driver_data,
             const PathData& ref_path = PathData());

    UserData(int param_flags,
             const d11::VehicleParam& veh_param,
             const d11::TMeasyNrParam& tire,
             const DriverData& driver_data,
             const PathData& ref_path = PathData());

    std::vector<realtype>& GetParams() { return m_params; }
    std::vector<realtype>& GetParamScales() { return m_param_scales; }

    const d11::VehicleParam& GetVehicleParam();
    const d11::TMeasyParam& GetTireTMParam();
    const d11::TMeasyNrParam& GetTireTMNrParam();

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

    d11::VehicleParam m_veh_param;
    d11::TMeasyParam m_tireTM_param;
    d11::TMeasyNrParam m_tireTMNr_param;
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

    friend class d11SolverSundials;
};

// =============================================================================

struct Objective {
    bool success;
    realtype value;
    std::vector<realtype> gradient;
};

#endif

class d11SolverSundials {
  public:
    d11SolverSundials();
    ~d11SolverSundials();

    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& driver_inputs_file);
    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& driver_inputs_file,
                   TireType tire_type);
    void SetTolerances(double rtol, double atol);
    void SetMaxStep(double hmax);
    void SetSensitivityParameters(int param_flags, const std::vector<double>& params);
    void SetSensitivityParameters(int param_flags, const double* params);
    void SetSensitivityParameterScales(const std::vector<double>& scales);
    void SetSensitivityParameterScales(const double* scales);
    void SetReferencePath(const std::string& reference_path_file);
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    void SetOutput(const std::string& output_file);

    int Initialize(d11::VehicleState& vehicle_states, d11::TMeasyState& tire_states_F, d11::TMeasyState& tire_states_R);
    int Initialize(d11::VehicleState& vehicle_states,
                   d11::TMeasyNrState& tire_states_F,
                   d11::TMeasyNrState& tire_states_R);

    int GetNumStates() const { return m_neq; }
    int GetNumParameters() const { return m_ns; }

    bool Solve(bool fsa);
    Objective Evaluate_FSA(bool gradient);

  private:
    bool Integrate(bool fsa);

    void Write(CSV_writer& csv, realtype t, N_Vector y, N_Vector* yS);

    bool m_has_TC;
    int m_offset;

    int m_method;     // CVODES integration method
    int m_mode;       // CVODES return mode
    UserData m_data;  // CVODES user data

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
    TireType m_tire_type;       // tire type
};

// =============================================================================

// Again don't need any of this in the python wrapper
#ifndef SWIG
void packY(const d11::VehicleState& v_states,
           const d11::TMeasyState& tiref_st,
           const d11::TMeasyState& tirer_st,
           bool has_TC,
           N_Vector& y);
void packY(const d11::VehicleState& v_states,
           const d11::TMeasyNrState& tiref_st,
           const d11::TMeasyNrState& tirer_st,
           bool has_TC,
           N_Vector& y);

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyState& tiref_st,
              const d11::TMeasyState& tirer_st,
              bool has_TC,
              N_Vector& ydot);

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyNrState& tiref_st,
              const d11::TMeasyNrState& tirer_st,
              bool has_TC,
              N_Vector& ydot);
void unpackY(const N_Vector& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyState& tiref_st,
             d11::TMeasyState& tirer_st);
void unpackY(const N_Vector& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyNrState& tiref_st,
             d11::TMeasyNrState& tirer_st);

void calcF();
void calcFtL();
void calcJtL();

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

#endif
#endif
