#ifndef DOF18_SUNDIALS_H
#define DOF18_SUNDIALS_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "dof18.h"

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
             const d18::VehicleParam& veh_param,
             const d18::TMeasyParam& tire,
             const DriverData& driver_data,
             const PathData& ref_path = PathData());

    std::vector<realtype>& GetParams() { return m_params; }
    std::vector<realtype>& GetParamScales() { return m_param_scales; }

    const d18::VehicleParam& GetVehicleParam();
    const d18::TMeasyParam& GetTireParam();
    const DriverData& GetDriverData() const { return m_driver_data; }
    const PathData& GetReferencePath() const { return m_ref_path; }
    bool HasReferencePath() const { return !m_ref_path.empty(); }

    void SetCurrentGear(int gear) { m_current_gr = gear; }
    int GetCurrentGear() const { return m_current_gr; }

    // Indices of vehicle position in state vectors
    int m_vx_idx;
    int m_vy_idx;

  private:
    bool IsFlagSet(ParamFlag::Enum val) const { return (m_param_flags & static_cast<int>(val)) != 0; }

    d18::VehicleParam m_veh_param;
    d18::TMeasyParam m_tire_param;
    DriverData m_driver_data;
    PathData m_ref_path;

    // Current transmission gear
    int m_current_gr;

    // Sensitivity parameters
    int m_param_flags;
    std::vector<realtype> m_params;
    std::vector<realtype> m_param_scales;

    friend class d18SolverSundials;
};

// =============================================================================

struct Objective {
    bool success;
    realtype value;
    std::vector<realtype> gradient;
};

#endif

class d18SolverSundials {
  public:
    d18SolverSundials();
    ~d18SolverSundials();

    void Construct(const std::string& vehicle_params_file,
                   const std::string& tire_params_file,
                   const std::string& driver_inputs_file);
    void SetTolerances(double rtol, double atol);
    void SetMaxStep(double hmax);
    void SetSensitivityParameters(int param_flags, const std::vector<double>& params);
    void SetSensitivityParameters(int param_flags, const double* params);
    void SetSensitivityParameterScales(const std::vector<double>& scales);
    void SetSensitivityParameterScales(const double* scales);
    void SetReferencePath(const std::string& reference_path_file);
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    void SetOutput(const std::string& output_file);

    int Initialize(d18::VehicleState& vehicle_states,
                   d18::TMeasyState& tire_states_LF,
                   d18::TMeasyState& tire_states_RF,
                   d18::TMeasyState& tire_states_LR,
                   d18::TMeasyState& tire_states_RR);

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
};

// =============================================================================

// Again don't need any of this in the python wrapper
#ifndef SWIG
void packY(const d18::VehicleState& v_states,
           const d18::TMeasyState& tirelf_st,
           const d18::TMeasyState& tirerf_st,
           const d18::TMeasyState& tirelr_st,
           const d18::TMeasyState& tirerr_st,
           bool has_TC,
           N_Vector& y);

void packYDOT(const d18::VehicleState& v_states,
              const d18::TMeasyState& tirelf_st,
              const d18::TMeasyState& tirerf_st,
              const d18::TMeasyState& tirelr_st,
              const d18::TMeasyState& tirerr_st,
              bool has_TC,
              N_Vector& ydot);

void unpackY(const N_Vector& y,
             bool has_TC,
             d18::VehicleState& v_states,
             d18::TMeasyState& tirelf_st,
             d18::TMeasyState& tirerf_st,
             d18::TMeasyState& tirelr_st,
             d18::TMeasyState& tirerr_st);

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
