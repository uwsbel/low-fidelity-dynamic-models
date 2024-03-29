#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof11.h"
#include "dof11_sundials.h"

using namespace d11;

// =============================================================================

UserData::UserData(int param_flags,
                   const VehicleParam& veh_param,
                   const TMeasyParam& tire,
                   const DriverData& driver_data,
                   const PathData& ref_path)
    : m_param_flags(param_flags),
      m_veh_param(veh_param),
      m_tireTM_param(tire),
      m_driver_data(driver_data),
      m_ref_path(ref_path) {
    if (IsFlagSet(ParamFlag::ENGINE_MAP)) {
        auto max_val = std::max_element(m_veh_param._powertrainMap.begin(),  //
                                        m_veh_param._powertrainMap.end(),    //
                                        [](const MapEntry& a, const MapEntry& b) { return abs(a._y) < abs(b._y); });
        for (const auto& entry : m_veh_param._powertrainMap) {
            m_params.push_back(entry._y);
            m_param_scales.push_back(max_val->_y);
        }
    }
    if (IsFlagSet(ParamFlag::STEERING_MAP)) {
        auto max_val = std::max_element(m_veh_param._steerMap.begin(),  //
                                        m_veh_param._steerMap.end(),    //
                                        [](const MapEntry& a, const MapEntry& b) { return abs(a._y) < abs(b._y); });
        for (const auto& entry : m_veh_param._steerMap) {
            m_params.push_back(entry._y);
            m_param_scales.push_back(max_val->_y);
        }
    }

    int offset = veh_param._tcbool ? 1 : 0;
    m_vx_idx = 6 + offset;
    m_vy_idx = 7 + offset;
}

UserData::UserData(int param_flags,
                   const VehicleParam& veh_param,
                   const TMeasyNrParam& tire,
                   const DriverData& driver_data,
                   const PathData& ref_path)
    : m_param_flags(param_flags),
      m_veh_param(veh_param),
      m_tireTMNr_param(tire),
      m_driver_data(driver_data),
      m_ref_path(ref_path) {
    if (IsFlagSet(ParamFlag::ENGINE_MAP)) {
        auto max_val = std::max_element(m_veh_param._powertrainMap.begin(),  //
                                        m_veh_param._powertrainMap.end(),    //
                                        [](const MapEntry& a, const MapEntry& b) { return abs(a._y) < abs(b._y); });
        for (const auto& entry : m_veh_param._powertrainMap) {
            m_params.push_back(entry._y);
            m_param_scales.push_back(max_val->_y);
        }
    }
    if (IsFlagSet(ParamFlag::STEERING_MAP)) {
        auto max_val = std::max_element(m_veh_param._steerMap.begin(),  //
                                        m_veh_param._steerMap.end(),    //
                                        [](const MapEntry& a, const MapEntry& b) { return abs(a._y) < abs(b._y); });
        for (const auto& entry : m_veh_param._steerMap) {
            m_params.push_back(entry._y);
            m_param_scales.push_back(max_val->_y);
        }
    }

    int offset = veh_param._tcbool ? 1 : 0;
    m_vx_idx = 2 + offset;
    m_vy_idx = 3 + offset;
}

const d11::VehicleParam& UserData::GetVehicleParam() {
    int index = 0;
    if (IsFlagSet(ParamFlag::ENGINE_MAP)) {
        for (auto& entry : m_veh_param._powertrainMap)
            entry._y = m_params[index++];
    }
    if (IsFlagSet(ParamFlag::STEERING_MAP)) {
        for (auto& entry : m_veh_param._steerMap)
            entry._y = m_params[index++];
    }
    return m_veh_param;
}

const d11::TMeasyParam& UserData::GetTireTMParam() {
    return m_tireTM_param;
}
const d11::TMeasyNrParam& UserData::GetTireTMNrParam() {
    return m_tireTMNr_param;
}

// =============================================================================
d11SolverSundials::d11SolverSundials()
    : m_method(CV_BDF),
      m_mode(CV_NORMAL),
      m_neq(0),
      m_ns(0),
      m_y0(nullptr),
      m_q0(nullptr),
      m_yS0(nullptr),
      m_qS0(nullptr),
      m_sunctx(nullptr),
      m_cvode_mem(nullptr),
      m_rtol(1e-5),
      m_atol(1e-3),
      m_hmax(1e6),
      m_output(false),
      m_verbose(false) {}

void d11SolverSundials::Construct(const std::string& vehicle_params_file,
                                  const std::string& tire_params_file,
                                  const std::string& driver_inputs_file) {
    // By default always TMEasy tire model is called
    m_tire_type = TireType::TMeasy;
    m_data.m_tire_type = TireType::TMeasy;  // Need to set this as well since we need it inside RHS
    // Load vehicle parameters
    setVehParamsJSON(m_data.m_veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(m_data.m_tireTM_param, tire_params_file.c_str());
    tireInit(m_data.m_tireTM_param);

    m_has_TC = m_data.m_veh_param._tcbool;
    m_offset = m_has_TC ? 1 : 0;
    m_data.m_vx_idx = 6 + m_offset;
    m_data.m_vy_idx = 7 + m_offset;

    // Load driver inputs
    LoadDriverData(m_data.m_driver_data, driver_inputs_file);

    // Cache final integration time
    m_tend = RCONST(m_data.m_driver_data.back().m_time);
}

void d11SolverSundials::Construct(const std::string& vehicle_params_file,
                                  const std::string& tire_params_file,
                                  const std::string& driver_inputs_file,
                                  TireType tire_type) {
    m_tire_type = tire_type;
    m_data.m_tire_type = tire_type;  // Need to set this as well since we need it inside RHS
    if (m_tire_type == TireType::TMeasy) {
        // Load vehicle parameters
        setVehParamsJSON(m_data.m_veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(m_data.m_tireTM_param, tire_params_file.c_str());
        tireInit(m_data.m_tireTM_param);
    } else if (m_tire_type == TireType::TMeasyNr) {
        // Load vehicle parameters
        setVehParamsJSON(m_data.m_veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(m_data.m_tireTMNr_param, tire_params_file.c_str());
        tireInit(m_data.m_tireTMNr_param);
    }

    m_has_TC = m_data.m_veh_param._tcbool;
    m_offset = m_has_TC ? 1 : 0;
    m_data.m_vx_idx = 2 + m_offset;
    m_data.m_vy_idx = 3 + m_offset;

    // Load driver inputs
    LoadDriverData(m_data.m_driver_data, driver_inputs_file);

    // Cache final integration time
    m_tend = RCONST(m_data.m_driver_data.back().m_time);
}

d11SolverSundials::~d11SolverSundials() {
    CVodeFree(&m_cvode_mem);

    N_VDestroy_Serial(m_y0);
    N_VDestroy_Serial(m_q0);
    N_VDestroyVectorArray(m_yS0, m_ns);
    N_VDestroyVectorArray(m_qS0, m_ns);

    SUNContext_Free(&m_sunctx);
}

void d11SolverSundials::SetMaxStep(double hmax) {
    m_hmax = RCONST(hmax);
}

void d11SolverSundials::SetTolerances(double rtol, double atol) {
    m_rtol = RCONST(rtol);
    m_atol = RCONST(atol);
}

void d11SolverSundials::SetSensitivityParameters(int param_flags, const std::vector<double>& params) {
    SetSensitivityParameters(param_flags, params.data());
}

void d11SolverSundials::SetSensitivityParameters(int param_flags, const double* params) {
    m_data.m_param_flags = param_flags;

    size_t num_params = 0;
    if (m_data.IsFlagSet(ParamFlag::ENGINE_MAP)) {
        num_params += m_data.m_veh_param._powertrainMap.size();
    }
    if (m_data.IsFlagSet(ParamFlag::STEERING_MAP)) {
        num_params += m_data.m_veh_param._steerMap.size();
    }

    m_ns = (int)num_params;
    m_data.m_params.resize(num_params);
    for (int is = 0; is < m_ns; is++) {
        m_data.m_params[is] = params[is];
    }

    m_data.m_param_scales.resize(num_params);
    size_t start_idx = 0;
    if (m_data.IsFlagSet(ParamFlag::ENGINE_MAP)) {
        size_t end_idx = m_data.m_veh_param._powertrainMap.size();
        auto max_val = std::max_element(m_data.m_params.begin() + start_idx,  //
                                        m_data.m_params.begin() + end_idx,    //
                                        [](const double& a, const double& b) { return abs(a) < abs(b); });
        std::fill(m_data.m_param_scales.begin() + start_idx, m_data.m_param_scales.begin() + end_idx, *max_val);
    }
}

void d11SolverSundials::SetSensitivityParameterScales(const std::vector<double>& scales) {
    SetSensitivityParameterScales(scales.data());
}

void d11SolverSundials::SetSensitivityParameterScales(const double* scales) {
    for (int is = 0; is < m_ns; is++) {
        m_data.m_param_scales[is] = scales[is];
    }
}

void d11SolverSundials::SetReferencePath(const std::string& reference_path_file) {
    LoadPath(m_data.m_ref_path, reference_path_file);
}

void d11SolverSundials::SetOutput(const std::string& output_file) {
    m_output = true;
    m_output_file = output_file;
}

int d11SolverSundials::Initialize(d11::VehicleState& vehicle_states,
                                  d11::TMeasyState& tire_states_F,
                                  d11::TMeasyState& tire_states_R) {
    // Cache initial transmission gear
    m_data.SetCurrentGear(vehicle_states._current_gr);

    // -------------------------------------------------------------------------

    int retval;

    // Create Sundials context
    retval = SUNContext_Create(NULL, &m_sunctx);
    if (check_retval(&retval, "SUNContext_Create", 1))
        return 1;

    // Create CVODE memory
    m_cvode_mem = CVodeCreate(m_method, m_sunctx);
    if (check_retval(m_cvode_mem, "CVodeCreate", 0))
        return 1;

    // Set user data
    CVodeSetUserData(m_cvode_mem, (void*)&m_data);
    if (check_retval(&retval, "CVodeSetUserData", 1))
        return 1;

    // -------------------------------------------------------------------------

    // Load initial conditions
    m_neq = m_data.m_veh_param._tcbool ? 13 : 12;
    m_y0 = N_VNew_Serial(m_neq, m_sunctx);
    packY(vehicle_states, tire_states_F, tire_states_R, m_has_TC, m_y0);

    // Initialize CVODES
    retval = CVodeInit(m_cvode_mem, rhsFun, ZERO, m_y0);
    if (check_retval(&retval, "CVodeInit", 1))
        return 1;

    // Set maximum number of steps between outputs
    retval = CVodeSetMaxNumSteps(m_cvode_mem, 50000);
    if (check_retval(&retval, "CVodeSetMaxNumSteps", 1))
        return 1;

    retval = CVodeSetMaxStep(m_cvode_mem, m_hmax);
    if (check_retval(&retval, "CVodeSetMaxStep", 1))
        return 1;

    // Set integration tolerances
    retval = CVodeSStolerances(m_cvode_mem, m_rtol, m_atol);

    ////CVodeSetMaxOrd(m_cvode_mem, 1);

    // -------------------------------------------------------------------------

    // Initialize quadrature
    if (m_data.HasReferencePath()) {
        m_q0 = N_VNew_Serial(1, m_sunctx);
        N_VConst(ZERO, m_q0);

        retval = CVodeQuadInit(m_cvode_mem, rhsQuad, m_q0);
        if (check_retval(&retval, "CVodeQuadInit", 1))
            return 1;

        retval = CVodeQuadSStolerances(m_cvode_mem, m_rtol, m_atol);
        retval = CVodeSetQuadErrCon(m_cvode_mem, SUNFALSE);
    }

    // -------------------------------------------------------------------------

    // Sensitivity initial conditions
    // Need this only for FSA -> Using reference path as a check
    if (m_data.HasReferencePath()) {
        m_yS0 = N_VCloneVectorArray(m_ns, m_y0);
        if (check_retval((void*)m_yS0, "N_VCloneVectorArray", 0))
            return 1;
        for (int is = 0; is < m_ns; is++)
            N_VConst(ZERO, m_yS0[is]);

        // Initialize state sensitivities
        retval = CVodeSensInit(m_cvode_mem, m_ns, CV_STAGGERED, NULL, m_yS0);
        if (check_retval(&retval, "CVodeSensInit", 1))
            return 1;

        // Use estimated tolerances for state sensitivities
        ////retval = CVodeSensEEtolerances(m_cvode_mem);
        ////if (check_retval(&retval, "CVodeSensEEtolerances", 1))
        ////    return 1;

        // Set tolerances for state sensitivities
        std::vector<realtype> atolS(m_ns, m_atol);
        retval = CVodeSensSStolerances(m_cvode_mem, m_rtol, atolS.data());

        // Error control strategy for sensitivity variables
        retval = CVodeSetSensErrCon(m_cvode_mem, SUNFALSE);
        if (check_retval(&retval, "CVodeSetSensErrCon", 1))
            return 1;

        // Specify problem sensitivity parameters
        retval = CVodeSetSensParams(m_cvode_mem, m_data.GetParams().data(), m_data.GetParamScales().data(), NULL);
        if (check_retval(&retval, "CVodeSetSensParams", 1))
            return 1;

        // -------------------------------------------------------------------------

        // Quadrature sensitivity initial conditions
        if (m_data.HasReferencePath()) {
            m_qS0 = N_VCloneVectorArray(m_ns, m_q0);
            if (check_retval((void*)m_qS0, "N_VCloneVectorArray", 0))
                return 1;
            for (int is = 0; is < m_ns; is++)
                N_VConst(ZERO, m_qS0[is]);

            // Initialize quadrature sensitivities
            retval = CVodeQuadSensInit(m_cvode_mem, rhsQuadSens, m_qS0);
            if (check_retval(&retval, "CVodeQuadSensInit", 1))
                return 1;

            ////retval = CVodeQuadSensEEtolerances(m_cvode_mem);
            ////if (check_retval(&retval, "CVodeQuadSensEEtolerances", 1))
            ////    return 1;

            retval = CVodeQuadSensSStolerances(m_cvode_mem, m_rtol, atolS.data());

            retval = CVodeSetQuadSensErrCon(m_cvode_mem, SUNFALSE);
            if (check_retval(&retval, "CVodeSetQuadSensErrCon", 1))
                return 1;
        }
    }

    // -------------------------------------------------------------------------

    // Set up nonlinear solver

    switch (m_method) {
        case CV_BDF: {
            // Create dense SUNMatrix for use in linear solves
            SUNMatrix A = SUNDenseMatrix(m_neq, m_neq, m_sunctx);
            if (check_retval((void*)A, "SUNDenseMatrix", 0))
                return 1;

            // Create dense SUNLinearSolver object
            SUNLinearSolver LS = SUNLinSol_Dense(m_y0, A, m_sunctx);
            if (check_retval((void*)LS, "SUNLinSol_Dense", 0))
                return 1;

            // Attach the matrix and linear solver to CVODE
            retval = CVodeSetLinearSolver(m_cvode_mem, LS, A);
            if (check_retval(&retval, "CVodeSetLinearSolver", 1))
                return 1;

            break;
        }

        case CV_ADAMS: {
            // Create fixed point nonlinear solver object
            SUNNonlinearSolver NLS = SUNNonlinSol_FixedPoint(m_y0, 0, m_sunctx);
            if (check_retval((void*)NLS, "SUNNonlinSol_FixedPoint", 0))
                return 1;

            // Attach nonlinear solver object to CVode
            retval = CVodeSetNonlinearSolver(m_cvode_mem, NLS);
            if (check_retval(&retval, "CVodeSetNonlinearSolver", 1))
                return 1;

            break;
        }

        default:
            return 1;
    }

    return 0;
}

int d11SolverSundials::Initialize(d11::VehicleState& vehicle_states,
                                  d11::TMeasyNrState& tire_states_F,
                                  d11::TMeasyNrState& tire_states_R) {
    // Cache initial transmission gear
    m_data.SetCurrentGear(vehicle_states._current_gr);

    // -------------------------------------------------------------------------

    int retval;

    // Create Sundials context
    retval = SUNContext_Create(NULL, &m_sunctx);
    if (check_retval(&retval, "SUNContext_Create", 1))
        return 1;

    // Create CVODE memory
    m_cvode_mem = CVodeCreate(m_method, m_sunctx);
    if (check_retval(m_cvode_mem, "CVodeCreate", 0))
        return 1;

    // Set user data
    CVodeSetUserData(m_cvode_mem, (void*)&m_data);
    if (check_retval(&retval, "CVodeSetUserData", 1))
        return 1;

    // -------------------------------------------------------------------------

    // Load initial conditions
    m_neq = m_data.m_veh_param._tcbool ? 9 : 8;
    m_y0 = N_VNew_Serial(m_neq, m_sunctx);
    packY(vehicle_states, tire_states_F, tire_states_R, m_has_TC, m_y0);

    // Initialize CVODES
    retval = CVodeInit(m_cvode_mem, rhsFun, ZERO, m_y0);
    if (check_retval(&retval, "CVodeInit", 1))
        return 1;

    // Set maximum number of steps between outputs
    retval = CVodeSetMaxNumSteps(m_cvode_mem, 50000);
    if (check_retval(&retval, "CVodeSetMaxNumSteps", 1))
        return 1;

    retval = CVodeSetMaxStep(m_cvode_mem, m_hmax);
    if (check_retval(&retval, "CVodeSetMaxStep", 1))
        return 1;

    // Set integration tolerances
    retval = CVodeSStolerances(m_cvode_mem, m_rtol, m_atol);

    ////CVodeSetMaxOrd(m_cvode_mem, 1);

    // -------------------------------------------------------------------------

    // Initialize quadrature
    if (m_data.HasReferencePath()) {
        m_q0 = N_VNew_Serial(1, m_sunctx);
        N_VConst(ZERO, m_q0);

        retval = CVodeQuadInit(m_cvode_mem, rhsQuad, m_q0);
        if (check_retval(&retval, "CVodeQuadInit", 1))
            return 1;

        retval = CVodeQuadSStolerances(m_cvode_mem, m_rtol, m_atol);
        retval = CVodeSetQuadErrCon(m_cvode_mem, SUNFALSE);
    }

    // -------------------------------------------------------------------------

    // Sensitivity initial conditions
    // Need this only for FSA -> Using reference path as a check
    if (m_data.HasReferencePath()) {
        m_yS0 = N_VCloneVectorArray(m_ns, m_y0);
        if (check_retval((void*)m_yS0, "N_VCloneVectorArray", 0))
            return 1;
        for (int is = 0; is < m_ns; is++)
            N_VConst(ZERO, m_yS0[is]);

        // Initialize state sensitivities
        retval = CVodeSensInit(m_cvode_mem, m_ns, CV_STAGGERED, NULL, m_yS0);
        if (check_retval(&retval, "CVodeSensInit", 1))
            return 1;

        // Use estimated tolerances for state sensitivities
        ////retval = CVodeSensEEtolerances(m_cvode_mem);
        ////if (check_retval(&retval, "CVodeSensEEtolerances", 1))
        ////    return 1;

        // Set tolerances for state sensitivities
        std::vector<realtype> atolS(m_ns, m_atol);
        retval = CVodeSensSStolerances(m_cvode_mem, m_rtol, atolS.data());

        // Error control strategy for sensitivity variables
        retval = CVodeSetSensErrCon(m_cvode_mem, SUNFALSE);
        if (check_retval(&retval, "CVodeSetSensErrCon", 1))
            return 1;

        // Specify problem sensitivity parameters
        retval = CVodeSetSensParams(m_cvode_mem, m_data.GetParams().data(), m_data.GetParamScales().data(), NULL);
        if (check_retval(&retval, "CVodeSetSensParams", 1))
            return 1;

        // -------------------------------------------------------------------------

        // Quadrature sensitivity initial conditions
        if (m_data.HasReferencePath()) {
            m_qS0 = N_VCloneVectorArray(m_ns, m_q0);
            if (check_retval((void*)m_qS0, "N_VCloneVectorArray", 0))
                return 1;
            for (int is = 0; is < m_ns; is++)
                N_VConst(ZERO, m_qS0[is]);

            // Initialize quadrature sensitivities
            retval = CVodeQuadSensInit(m_cvode_mem, rhsQuadSens, m_qS0);
            if (check_retval(&retval, "CVodeQuadSensInit", 1))
                return 1;

            ////retval = CVodeQuadSensEEtolerances(m_cvode_mem);
            ////if (check_retval(&retval, "CVodeQuadSensEEtolerances", 1))
            ////    return 1;

            retval = CVodeQuadSensSStolerances(m_cvode_mem, m_rtol, atolS.data());

            retval = CVodeSetQuadSensErrCon(m_cvode_mem, SUNFALSE);
            if (check_retval(&retval, "CVodeSetQuadSensErrCon", 1))
                return 1;
        }
    }

    // -------------------------------------------------------------------------

    // Set up nonlinear solver

    switch (m_method) {
        case CV_BDF: {
            // Create dense SUNMatrix for use in linear solves
            SUNMatrix A = SUNDenseMatrix(m_neq, m_neq, m_sunctx);
            if (check_retval((void*)A, "SUNDenseMatrix", 0))
                return 1;

            // Create dense SUNLinearSolver object
            SUNLinearSolver LS = SUNLinSol_Dense(m_y0, A, m_sunctx);
            if (check_retval((void*)LS, "SUNLinSol_Dense", 0))
                return 1;

            // Attach the matrix and linear solver to CVODE
            retval = CVodeSetLinearSolver(m_cvode_mem, LS, A);
            if (check_retval(&retval, "CVodeSetLinearSolver", 1))
                return 1;

            break;
        }

        case CV_ADAMS: {
            // Create fixed point nonlinear solver object
            SUNNonlinearSolver NLS = SUNNonlinSol_FixedPoint(m_y0, 0, m_sunctx);
            if (check_retval((void*)NLS, "SUNNonlinSol_FixedPoint", 0))
                return 1;

            // Attach nonlinear solver object to CVode
            retval = CVodeSetNonlinearSolver(m_cvode_mem, NLS);
            if (check_retval(&retval, "CVodeSetNonlinearSolver", 1))
                return 1;

            break;
        }

        default:
            return 1;
    }

    return 0;
}

bool d11SolverSundials::Solve(bool fsa) {
    int retval;

    // Reinitialize solver
    CVodeReInit(m_cvode_mem, 0, m_y0);
    if (fsa) {
        retval = CVodeSensReInit(m_cvode_mem, CV_STAGGERED, m_yS0);
        if (check_retval(&retval, "CVodeSensReInit", 1))
            return false;
    } else {
        retval = CVodeSensToggleOff(m_cvode_mem);
        if (check_retval(&retval, "CVodeSensToggleOff", 1))
            return false;
    }

    // Integrate to final time
    if (!Integrate(fsa))
        return false;

    return true;
}

Objective d11SolverSundials::Evaluate_FSA(bool gradient) {
    assert(m_data.HasReferencePath());

    Objective cf = {false, 0.0, std::vector<realtype>()};

    int retval;

    // Reinitialize solver
    CVodeReInit(m_cvode_mem, 0, m_y0);
    CVodeQuadReInit(m_cvode_mem, m_q0);
    if (gradient) {
        retval = CVodeSensReInit(m_cvode_mem, CV_STAGGERED, m_yS0);
        if (check_retval(&retval, "CVodeSensReInit", 1))
            return cf;
        retval = CVodeQuadSensReInit(m_cvode_mem, m_qS0);
        if (check_retval(&retval, "CVodeQuadSensReInit", 1))
            return cf;
    } else {
        retval = CVodeSensToggleOff(m_cvode_mem);
        if (check_retval(&retval, "CVodeSensToggleOff", 1))
            return cf;
    }

    // Integrate to final time
    if (!Integrate(gradient))
        return cf;

    // Extract quadrature value and gradient (if applicable)
    realtype t;
    N_Vector q = N_VNew_Serial(1, m_sunctx);
    N_Vector* qS = N_VCloneVectorArray(m_ns, q);

    retval = CVodeGetQuad(m_cvode_mem, &t, q);
    if (check_retval(&retval, "CVodeGetQuad", 1))
        return cf;
    cf.value = Ith(q, 0);

    if (gradient) {
        retval = CVodeGetQuadSens(m_cvode_mem, &t, qS);
        if (check_retval(&retval, "CVodeGetQuadSens", 1))
            return cf;
        for (int is = 0; is < m_ns; is++)
            cf.gradient.push_back(Ith(qS[is], 0));
    }

    cf.success = true;

    N_VDestroy_Serial(q);
    N_VDestroyVectorArray(qS, m_ns);

    return cf;
}

bool d11SolverSundials::Integrate(bool fsa) {
    realtype dtout = 0.01;  // 10 Hz output frequency
    realtype t = 0;
    int retval;

    // Create output writer
    CSV_writer csv(" ");
    if (m_output) {
        Write(csv, t, m_y0, fsa ? m_yS0 : nullptr);
    }

    // Integrate to final time
    N_Vector y = N_VNew_Serial(m_neq, m_sunctx);
    N_Vector* yS = fsa ? N_VCloneVectorArray(m_ns, y) : nullptr;

    switch (m_mode) {
        case CV_NORMAL: {
            realtype nout = m_tend / dtout + 1;

            for (int iout = 1; iout < nout; iout++) {
                retval = CVode(m_cvode_mem, iout * dtout, y, &t, CV_NORMAL);
                if (check_retval(&retval, "CVode", 1))
                    return false;
                SUNMatrix J = SUNDenseMatrix(m_neq, m_neq, m_sunctx);
                // Get the RHS jacobian and print only in debug mode
                if (m_verbose) {
                    CVodePrintAllStats(m_cvode_mem, stdout, SUN_OUTPUTFORMAT_TABLE);
                    printf("\n");
                }

                if (m_output) {
                    if (fsa)
                        CVodeGetSens(m_cvode_mem, &t, yS);
                    Write(csv, t, y, yS);
                }
            }

            break;
        }

        case CV_ONE_STEP: {
            while (t < m_tend) {
                retval = CVode(m_cvode_mem, m_tend, y, &t, CV_ONE_STEP);
                if (check_retval(&retval, "CVode", 1))
                    return false;

                if (m_verbose) {
                    CVodePrintAllStats(m_cvode_mem, stdout, SUN_OUTPUTFORMAT_TABLE);
                    printf("\n");
                }

                if (m_output) {
                    if (fsa)
                        CVodeGetSens(m_cvode_mem, &t, yS);
                    Write(csv, t, y, yS);
                }
            }
            break;
        }
    }

    CVodePrintAllStats(m_cvode_mem, stdout, SUN_OUTPUTFORMAT_TABLE);

    if (m_output)
        csv.write_to_file(m_output_file);

    N_VDestroy_Serial(y);

    return true;
}

void d11SolverSundials::Write(CSV_writer& csv, realtype t, N_Vector y, N_Vector* yS) {
    csv << t;
    if (m_tire_type == TireType::TMeasy) {
        csv << Ith(y, 6 + m_offset) << Ith(y, 7 + m_offset) << Ith(y, 8 + m_offset) << Ith(y, 9 + m_offset)
            << Ith(y, 10 + m_offset) << Ith(y, 11 + m_offset);
    } else {
        csv << Ith(y, 2 + m_offset) << Ith(y, 3 + m_offset) << Ith(y, 4 + m_offset) << Ith(y, 5 + m_offset)
            << Ith(y, 6 + m_offset) << Ith(y, 7 + m_offset);
    }

    ////csv << Ith(y, 14 + m_offset) << Ith(y, 15 + m_offset);
    ////csv << Ith(y, 8);
    if (yS) {
        for (int is = 0; is < m_ns; is++)
            if (m_tire_type == TireType::TMeasy) {
                csv << Ith(yS[is], 6 + m_offset) << Ith(yS[is], 7 + m_offset);
            } else {
                csv << Ith(yS[is], 2 + m_offset) << Ith(yS[is], 3 + m_offset);
            }
    }
    csv << std::endl;
}

// =============================================================================

// Function to evaluate f(t, y, ydot) to be used by Sundials ODE solver
int d11::rhsFun(realtype t, N_Vector y, N_Vector ydot, void* user_data) {
    ////std::cout << "RHS t = " << t << std::endl;

    // Unpack user data
    UserData* udata = (UserData*)user_data;
    const VehicleParam& veh_param = udata->GetVehicleParam();
    if (udata->GetTireType() == TireType::TMeasy) {
        const TMeasyParam& tire_param = udata->GetTireTMParam();

        // Extract vehicle and tire state from CVODES state
        VehicleState veh_st;
        TMeasyState tiref_st;
        TMeasyState tirer_st;

        unpackY(y, veh_param._tcbool, veh_st, tiref_st, tirer_st);

        // Keep track of the current gear
        veh_st._current_gr = udata->GetCurrentGear();

        // First get the controls at the current timeStep
        auto controls = GetDriverInput(t, udata->GetDriverData());

        // Calculate tire vertical loads
        std::vector<double> loads(2, 0);
        computeTireLoads(loads, veh_st, veh_param, tire_param);

        // Transform from the vehicle frame to the tire frame
        vehToTireTransform(tiref_st, tirer_st, veh_st, loads, veh_param, controls.m_steering);

        // Tire dynamics
        computeTireRHS(tiref_st, tire_param, veh_param, controls.m_steering);

        computeTireRHS(tirer_st, tire_param, veh_param, 0);

        // Powertrain dynamics
        computePowertrainRHS(veh_st, tiref_st, tirer_st, veh_param, tire_param, controls);

        // Vehicle dynamics
        tireToVehTransform(tiref_st, tirer_st, veh_st, veh_param, controls.m_steering);
        std::vector<double> fx = {tiref_st._fx, tirer_st._fx};
        std::vector<double> fy = {tiref_st._fy, tirer_st._fy};
        computeVehRHS(veh_st, veh_param, fx, fy);

        // Load RHS
        packYDOT(veh_st, tiref_st, tirer_st, veh_param._tcbool, ydot);

        // Keeping track of the current gear
        udata->SetCurrentGear(veh_st._current_gr);
    } else if (udata->GetTireType() == TireType::TMeasyNr) {
        const TMeasyNrParam& tire_param = udata->GetTireTMNrParam();

        // Extract vehicle and tire state from CVODES state
        VehicleState veh_st;
        TMeasyNrState tiref_st;
        TMeasyNrState tirer_st;

        unpackY(y, veh_param._tcbool, veh_st, tiref_st, tirer_st);

        // Keep track of the current gear
        veh_st._current_gr = udata->GetCurrentGear();

        // First get the controls at the current timeStep
        auto controls = GetDriverInput(t, udata->GetDriverData());

        // Calculate tire vertical loads
        std::vector<double> loads(2, 0);
        computeTireLoads(loads, veh_st, veh_param, tire_param);

        // Transform from the vehicle frame to the tire frame
        vehToTireTransform(tiref_st, tirer_st, veh_st, loads, veh_param, controls.m_steering);

        // Tire dynamics
        computeTireRHS(tiref_st, tire_param, veh_param, controls.m_steering);

        computeTireRHS(tirer_st, tire_param, veh_param, 0);

        // Powertrain dynamics
        computePowertrainRHS(veh_st, tiref_st, tirer_st, veh_param, tire_param, controls);

        // Vehicle dynamics
        tireToVehTransform(tiref_st, tirer_st, veh_st, veh_param, controls.m_steering);
        std::vector<double> fx = {tiref_st._fx, tirer_st._fx};
        std::vector<double> fy = {tiref_st._fy, tirer_st._fy};
        computeVehRHS(veh_st, veh_param, fx, fy);

        // Load RHS
        packYDOT(veh_st, tiref_st, tirer_st, veh_param._tcbool, ydot);

        // Keeping track of the current gear
        udata->SetCurrentGear(veh_st._current_gr);
    }
    return 0;
}

int d11::rhsQuad(realtype t, N_Vector y, N_Vector qdot, void* user_data) {
    // Unpack user data
    UserData* udata = (UserData*)user_data;
    const PathData& ref_path = udata->GetReferencePath();

    // Current x,y vehicle position
    double x_crt = Ith(y, udata->m_vx_idx);
    double y_crt = Ith(y, udata->m_vy_idx);

    // Reference position
    PathPoint ref = GetPathPoint(t, ref_path);

    // Evaluate integrand at current time
    Ith(qdot, 0) = (x_crt - ref.m_x) * (x_crt - ref.m_x) + (y_crt - ref.m_y) * (y_crt - ref.m_y);

    return 0;
}

int d11::rhsQuadSens(int Ns,
                     realtype t,
                     N_Vector y,
                     N_Vector* yS,
                     N_Vector qdot,
                     N_Vector* qSdot,
                     void* user_data,
                     N_Vector tmp,
                     N_Vector tmpQ) {
    // Unpack user data
    UserData* udata = (UserData*)user_data;
    const VehicleParam& veh_param = udata->GetVehicleParam();
    const PathData& ref_path = udata->GetReferencePath();

    // Current x,y vehicle position
    double x_crt = Ith(y, udata->m_vx_idx);
    double y_crt = Ith(y, udata->m_vy_idx);

    // Reference position
    PathPoint ref = GetPathPoint(t, ref_path);

    // Evaluate gradient of integrand at current time
    for (int is = 0; is < Ns; is++) {
        double xs_crt = Ith(yS[is], udata->m_vx_idx);
        double ys_crt = Ith(yS[is], udata->m_vy_idx);
        Ith(qSdot[is], 0) = 2 * (x_crt - ref.m_x) * xs_crt + 2 * (y_crt - ref.m_y) * ys_crt;
    }

    return 0;
}

// =============================================================================

void packY(const d11::VehicleState& v_states,
           const d11::TMeasyState& tiref_st,
           const d11::TMeasyState& tirer_st,
           bool has_TC,
           N_Vector& y) {
    int index = 0;

    // Tire deflections (lf, rf, lr and rr)
    Ith(y, index++) = tiref_st._xe;
    Ith(y, index++) = tiref_st._ye;
    Ith(y, index++) = tirer_st._xe;
    Ith(y, index++) = tirer_st._ye;

    // Wheel angular velocities (lf, rf, lr and rr)
    Ith(y, index++) = tiref_st._omega;
    Ith(y, index++) = tirer_st._omega;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        Ith(y, index++) = v_states._crankOmega;
    }

    // Vehicle states
    Ith(y, index++) = v_states._x;    // X position
    Ith(y, index++) = v_states._y;    // Y position
    Ith(y, index++) = v_states._u;    // longitudinal velocity
    Ith(y, index++) = v_states._v;    // lateral velocity
    Ith(y, index++) = v_states._psi;  // yaw angle
    Ith(y, index++) = v_states._wz;   // yaw rate
}

void packY(const d11::VehicleState& v_states,
           const d11::TMeasyNrState& tiref_st,
           const d11::TMeasyNrState& tirer_st,
           bool has_TC,
           N_Vector& y) {
    int index = 0;
    // Wheel angular velocities (lf, rf, lr and rr)
    Ith(y, index++) = tiref_st._omega;
    Ith(y, index++) = tirer_st._omega;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        Ith(y, index++) = v_states._crankOmega;
    }

    // Vehicle states
    Ith(y, index++) = v_states._x;    // X position
    Ith(y, index++) = v_states._y;    // Y position
    Ith(y, index++) = v_states._u;    // longitudinal velocity
    Ith(y, index++) = v_states._v;    // lateral velocity
    Ith(y, index++) = v_states._psi;  // yaw angle
    Ith(y, index++) = v_states._wz;   // yaw rate
}

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyState& tiref_st,
              const d11::TMeasyState& tirer_st,
              bool has_TC,
              N_Vector& ydot) {
    int index = 0;

    Ith(ydot, index++) = tiref_st._xedot;
    Ith(ydot, index++) = tiref_st._yedot;
    Ith(ydot, index++) = tirer_st._xedot;
    Ith(ydot, index++) = tirer_st._yedot;

    Ith(ydot, index++) = tiref_st._dOmega;
    Ith(ydot, index++) = tirer_st._dOmega;

    if (has_TC) {
        Ith(ydot, index++) = v_states._dOmega_crank;
    }

    Ith(ydot, index++) = v_states._dx;
    Ith(ydot, index++) = v_states._dy;
    Ith(ydot, index++) = v_states._udot;
    Ith(ydot, index++) = v_states._vdot;
    Ith(ydot, index++) = v_states._wz;
    Ith(ydot, index++) = v_states._wzdot;
}

void packYDOT(const d11::VehicleState& v_states,
              const d11::TMeasyNrState& tiref_st,
              const d11::TMeasyNrState& tirer_st,
              bool has_TC,
              N_Vector& ydot) {
    int index = 0;
    Ith(ydot, index++) = tiref_st._dOmega;
    Ith(ydot, index++) = tirer_st._dOmega;

    if (has_TC) {
        Ith(ydot, index++) = v_states._dOmega_crank;
    }

    Ith(ydot, index++) = v_states._dx;
    Ith(ydot, index++) = v_states._dy;
    Ith(ydot, index++) = v_states._udot;
    Ith(ydot, index++) = v_states._vdot;
    Ith(ydot, index++) = v_states._wz;
    Ith(ydot, index++) = v_states._wzdot;
}

void unpackY(const N_Vector& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyState& tiref_st,
             d11::TMeasyState& tirer_st) {
    int index = 0;

    // Tire deflections
    tiref_st._xe = Ith(y, index++);
    tiref_st._ye = Ith(y, index++);
    tirer_st._xe = Ith(y, index++);
    tirer_st._ye = Ith(y, index++);

    // Wheel angular velocities
    tiref_st._omega = Ith(y, index++);
    tirer_st._omega = Ith(y, index++);

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        v_states._crankOmega = Ith(y, index++);
    }

    // Vehicle states
    v_states._x = Ith(y, index++);    // X position
    v_states._y = Ith(y, index++);    // Y position
    v_states._u = Ith(y, index++);    // longitudinal velocity
    v_states._v = Ith(y, index++);    // lateral velocity
    v_states._psi = Ith(y, index++);  // yaw angle
    v_states._wz = Ith(y, index++);   // yaw rate
}

void unpackY(const N_Vector& y,
             bool has_TC,
             d11::VehicleState& v_states,
             d11::TMeasyNrState& tiref_st,
             d11::TMeasyNrState& tirer_st) {
    int index = 0;
    // Wheel angular velocities
    tiref_st._omega = Ith(y, index++);
    tirer_st._omega = Ith(y, index++);

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        v_states._crankOmega = Ith(y, index++);
    }

    // Vehicle states
    v_states._x = Ith(y, index++);    // X position
    v_states._y = Ith(y, index++);    // Y position
    v_states._u = Ith(y, index++);    // longitudinal velocity
    v_states._v = Ith(y, index++);    // lateral velocity
    v_states._psi = Ith(y, index++);  // yaw angle
    v_states._wz = Ith(y, index++);   // yaw rate
}

// =============================================================================

int d11::printStatsCvode(void* cvode_mem) {
    int retval = 0;
    long int nst, nfe, nsetups, nje, nfeLS, ncfn, netf;

    retval = CVodeGetNumSteps(cvode_mem, &nst);
    if (check_retval(&retval, "CVodeGetNumSteps", 1))
        return retval;

    retval = CVodeGetNumRhsEvals(cvode_mem, &nfe);
    if (check_retval(&retval, "CVodeGetNumFctEvals", 1))
        return retval;

    retval = CVodeGetNumLinSolvSetups(cvode_mem, &nsetups);
    if (check_retval(&retval, "CVodeGetNumLinSolvSetups", 1))
        return retval;

    retval = CVodeGetNumErrTestFails(cvode_mem, &netf);
    if (check_retval(&retval, "CVodeGetNumErrTestFails", 1))
        return retval;

    retval = CVodeGetNumNonlinSolvConvFails(cvode_mem, &ncfn);
    if (check_retval(&retval, "CVodeGetNumNonlinSolvConvFails", 1))
        return retval;

    retval = CVodeGetNumJacEvals(cvode_mem, &nje);
    if (check_retval(&retval, "CVodeGetNumJacEvals", 1))
        return retval;

    retval = CVodeGetNumLinRhsEvals(cvode_mem, &nfeLS);
    if (check_retval(&retval, "CVodeGetNumLinRhsEvals", 1))
        return retval;

    /* Output stats */
    printf(" %6ld   %6ld+%-4ld     %4ld (%3ld)     |  %3ld  %3ld\n", nst, nfe, nfeLS, nsetups, nje, ncfn, netf);

    return retval;
}

void d11::printSUNMatrix(SUNMatrix A, sunindextype matrows, sunindextype matcols) {
    realtype* Adata = SUNDenseMatrix_Data(A);
    std::cout << "Jacobian state: " << std::endl;
    for (sunindextype i = 0; i < matrows; i++) {
        for (sunindextype j = 0; j < matcols; j++) {
            printf("%.4f  ", Adata[j * matrows + i]);
        }
        printf("\n");
    }
}