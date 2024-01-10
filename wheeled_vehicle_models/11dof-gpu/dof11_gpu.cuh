#ifndef DOF11_GPU_CUH
#define DOF11_GPU_CUH

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "utils_gpu.cuh"
// enum to decide what type of tire we have
enum class TireType { TMeasy, TMeasyNr };
namespace d11GPU {

// TMeasy parameter structure
struct TMeasyParam {
    // constructor that takes default values of HMMWV
    __device__ __host__ TMeasyParam()
        : _jw(6.69),
          _rr(0.015),
          _mu(0.8),
          _r0(0.4699),
          _pn(8562.8266),
          _pnmax(29969.893),
          _cx(185004.42),
          _cy(164448.37),
          _kt(411121.0),
          _dx(3700.),
          _dy(3488.),
          _rdyncoPn(0.375),
          _rdyncoP2n(0.75),
          _fzRdynco(0),
          _rdyncoCrit(0),
          _dfx0Pn(151447.29),
          _dfx0P2n(236412.79),
          _fxmPn(7575.3614),
          _fxmP2n(12808.276),
          _fxsPn(4657.9208),
          _fxsP2n(8625.3352),
          _sxmPn(0.12),
          _sxmP2n(0.15),
          _sxsPn(0.9),
          _sxsP2n(0.95),
          _dfy0Pn(50931.693),
          _dfy0P2n(94293.847),
          _fymPn(6615.0404),
          _fymP2n(12509.947),
          _fysPn(6091.5092),
          _fysP2n(11443.875),
          _symPn(0.38786),
          _symP2n(0.38786),
          _sysPn(0.82534),
          _sysP2n(0.91309) {}
    // copy constructorother->_
    __device__ __host__ TMeasyParam(const TMeasyParam* other)
        : _jw(other->_jw),
          _rr(other->_rr),
          _mu(other->_mu),
          _r0(other->_r0),
          _pn(other->_pn),
          _pnmax(other->_pnmax),
          _cx(other->_cx),
          _cy(other->_cy),
          _kt(other->_kt),
          _dx(other->_dx),
          _dy(other->_dy),
          _rdyncoPn(other->_rdyncoPn),
          _rdyncoP2n(other->_rdyncoP2n),
          _fzRdynco(other->_fzRdynco),
          _rdyncoCrit(other->_rdyncoCrit),
          _dfx0Pn(other->_dfx0Pn),
          _dfx0P2n(other->_dfx0P2n),
          _fxmPn(other->_fxmPn),
          _fxmP2n(other->_fxmP2n),
          _fxsPn(other->_fxsPn),
          _fxsP2n(other->_fxsP2n),
          _sxmPn(other->_sxmPn),
          _sxmP2n(other->_sxmP2n),
          _sxsPn(other->_sxsPn),
          _sxsP2n(other->_sxsP2n),
          _dfy0Pn(other->_dfy0Pn),
          _dfy0P2n(other->_dfy0P2n),
          _fymPn(other->_fymPn),
          _fymP2n(other->_fymP2n),
          _fysPn(other->_fysPn),
          _fysP2n(other->_fysP2n),
          _symPn(other->_symPn),
          _symP2n(other->_symP2n),
          _sysPn(other->_sysPn),
          _sysP2n(other->_sysP2n) {}

    // basic tire parameters
    double _jw;  // wheel inertia
    double _rr;  // rolling resistance of tire
    double _mu;  // friction constant
    double _r0;  // unloaded tire radius

    // TM easy specific tire params
    double _pn, _pnmax;    // nominal and max vertical force
    double _cx, _cy, _kt;  // longitudinal, lateral and vertical stiffness
    double _dx,
        _dy;  // longitudinal and lateral damping coeffs. No vertical damping

    // TM easy force characteristic params
    // 2 values - one at nominal load and one at max load

    // dynamic radius weighting coefficient and a critical value for the
    // vertical force
    double _rdyncoPn, _rdyncoP2n, _fzRdynco, _rdyncoCrit;

    // Longitudinal
    double _dfx0Pn, _dfx0P2n;  // intial longitudinal slopes dFx/dsx [N]
    double _fxmPn, _fxmP2n;    // maximum longituidnal force [N]
    double _fxsPn, _fxsP2n;    // Longitudinal load at sliding [N]
    double _sxmPn, _sxmP2n;    // slip sx at maximum longitudinal load Fx
    double _sxsPn, _sxsP2n;    // slip sx where sliding begins
    // Lateral
    double _dfy0Pn, _dfy0P2n;  // intial lateral slopes dFx/dsx [N]
    double _fymPn, _fymP2n;    // maximum lateral force [N]
    double _fysPn, _fysP2n;    // Lateral load at sliding [N]
    double _symPn, _symP2n;    // slip sx at maximum lateral load Fx
    double _sysPn, _sysP2n;    // slip sx where sliding begins
};

// Tm easy state structure - actual states + things that we need to keep track
// of
struct TMeasyState {
    // default contructor to 0's
    __device__ __host__ TMeasyState()
        : _xe(0.),
          _ye(0.),
          _xedot(0.),
          _yedot(0.),
          _omega(0.),
          _dOmega(0),
          _xt(0.),
          _rStat(0.),
          _fx(0.),
          _fy(0.),
          _fz(0.),
          _vsx(0.),
          _vsy(0.),
          _My(0.),
          _engTor(0.) {}

    // special constructor in case we want to start the simualtion at
    // some other time step
    __device__ __host__ TMeasyState(double xe,
                                    double ye,
                                    double xedot,
                                    double yedot,
                                    double omega,
                                    double xt,
                                    double rStat,
                                    double fx,
                                    double fy,
                                    double fz,
                                    double vsx,
                                    double vsy,
                                    double init_ratio)
        : _xe(xe),
          _ye(ye),
          _xedot(xedot),
          _yedot(yedot),
          _omega(omega),
          _xt(xt),
          _rStat(rStat),
          _fx(fx),
          _fy(fy),
          _fz(fz),
          _vsx(vsx),
          _vsy(vsy) {}

    // Copy constructor
    __device__ __host__ TMeasyState(const TMeasyState* other)
        : _xe(other->_xe),
          _ye(other->_ye),
          _xedot(other->_xedot),
          _yedot(other->_yedot),
          _omega(other->_omega),
          _dOmega(other->_dOmega),
          _xt(other->_xt),
          _rStat(other->_rStat),
          _fx(other->_fx),
          _fy(other->_fy),
          _fz(other->_fz),
          _vsx(other->_vsx),
          _vsy(other->_vsy),
          _My(other->_My),
          _engTor(other->_engTor) {}

    // the actual state that are intgrated
    double _xe, _ye;        // long and lat tire deflection
    double _xedot, _yedot;  // long and lat tire deflection velocity
    double _omega;          // angular velocity of wheel
    double _dOmega;         // angular velocity dot

    // other "states" that we need to keep track of
    double _xt;            // vertical tire compression
    double _rStat;         // loaded tire radius
    double _fx, _fy, _fz;  // long, lateral and vertical force in tire frame

    // velocities in tire frame
    double _vsx, _vsy;

    double _My;  // Rolling resistance moment (negetive)

    // torque from engine that we keep track of
    double _engTor;
};

// TMeasy tire without relaxation
struct TMeasyNrParam {
    // default constructor just assigns zero to all members
    __device__ __host__ TMeasyNrParam()
        : _jw(6.69),
          _rr(0.015),
          _mu(0.8),
          _r0(0.4699),
          _width(0.245),
          _rim_radius(0.254),
          _pn(8562.8266),
          _pnmax(29969.893),
          _kt(411121.0),
          _rdyncoPn(0.375),
          _rdyncoP2n(0.75),
          _fzRdynco(0),
          _rdyncoCrit(0),
          _dfx0Pn(151447.29),
          _dfx0P2n(236412.79),
          _fxmPn(7575.3614),
          _fxmP2n(12808.276),
          _fxsPn(4657.9208),
          _fxsP2n(8625.3352),
          _sxmPn(0.12),
          _sxmP2n(0.15),
          _sxsPn(0.9),
          _sxsP2n(0.95),
          _dfy0Pn(50931.693),
          _dfy0P2n(94293.847),
          _fymPn(6615.0404),
          _fymP2n(12509.947),
          _fysPn(6091.5092),
          _fysP2n(11443.875),
          _symPn(0.38786),
          _symP2n(0.38786),
          _sysPn(0.82534),
          _sysP2n(0.91309),
          _vcoulomb(1.0),
          _frblend_begin(1.),
          _frblend_end(3.),
          _bearingCapacity(10000),
          _li(90),
          _p_li(1.),
          _p_use(1.) {}
    // copy constructor
    __device__ __host__ TMeasyNrParam(const TMeasyNrParam* other)
        : _jw(other->_jw),
          _rr(other->_rr),
          _mu(other->_mu),
          _r0(other->_r0),
          _width(other->_width),
          _rim_radius(other->_rim_radius),
          _pn(other->_pn),
          _pnmax(other->_pnmax),
          _kt(other->_kt),
          _rdyncoPn(other->_rdyncoPn),
          _rdyncoP2n(other->_rdyncoP2n),
          _fzRdynco(other->_fzRdynco),
          _rdyncoCrit(other->_rdyncoCrit),
          _dfx0Pn(other->_dfx0Pn),
          _dfx0P2n(other->_dfx0P2n),
          _fxmPn(other->_fxmPn),
          _fxmP2n(other->_fxmP2n),
          _fxsPn(other->_fxsPn),
          _fxsP2n(other->_fxsP2n),
          _sxmPn(other->_sxmPn),
          _sxmP2n(other->_sxmP2n),
          _sxsPn(other->_sxsPn),
          _sxsP2n(other->_sxsP2n),
          _dfy0Pn(other->_dfy0Pn),
          _dfy0P2n(other->_dfy0P2n),
          _fymPn(other->_fymPn),
          _fymP2n(other->_fymP2n),
          _fysPn(other->_fysPn),
          _fysP2n(other->_fysP2n),
          _symPn(other->_symPn),
          _symP2n(other->_symP2n),
          _sysPn(other->_sysPn),
          _sysP2n(other->_sysP2n),
          _vcoulomb(other->_vcoulomb),
          _frblend_begin(other->_frblend_begin),
          _frblend_end(other->_frblend_end),
          _bearingCapacity(other->_bearingCapacity),
          _li(other->_li),
          _p_li(other->_p_li),
          _p_use(other->_p_use) {}

    double _jw;          // wheel inertia
    double _rr;          // Rolling Resistance
    double _mu;          // Local friction coefficient between tire and road
    double _r0;          // unloaded tire radius
    double _width;       // tire width
    double _rim_radius;  // rim radius
    double _pn;          // nominal vertical force
    double _pnmax;       // max vertical force
    double _kt;          // vertical tire stiffness
    double _rdyncoPn;    // dynamic radius weighting coefficient at nominal load
    double _rdyncoP2n;   // dynamic radius weighting coefficient at max load
    double _fzRdynco;    // critical value for the vertical force
    double _rdyncoCrit;
    // Longitudinal
    double _dfx0Pn, _dfx0P2n;  // Initial longitudinal slopes dFx/dsx [N]
    double _fxmPn, _fxmP2n;    // Maximum longituidnal force [N]
    double _fxsPn, _fxsP2n;    // Longitudinal load at sliding [N]
    double _sxmPn, _sxmP2n;    // slip sx at maximum longitudinal load Fx
    double _sxsPn, _sxsP2n;    // slip sx where sliding begins
    // Lateral
    double _dfy0Pn, _dfy0P2n;  // intial lateral slopes dFx/dsx [N]
    double _fymPn, _fymP2n;    // maximum lateral force [N]
    double _fysPn, _fysP2n;    // Lateral load at sliding [N]
    double _symPn, _symP2n;    // slip sx at maximum lateral load Fx
    double _sysPn, _sysP2n;    // slip sx where sliding begins
    double _vcoulomb;          // Velocity below which we care about static friction
    double _frblend_begin;     // Beginning of friction blending
    double _frblend_end;       // End of friction blending
    double _bearingCapacity;   // High level tire parameters that define all other parameters that the user can set
    double _li;                // Load index
    double _p_li;              // Pressure at load index
    double _p_use;             // Pressure at which the tire is used
};

struct TMeasyNrState {
    __device__ __host__ TMeasyNrState()
        : _omega(0.),
          _dOmega(0),
          _xt(0.),
          _rStat(0.),
          _fx(0.),
          _fy(0.),
          _fz(0.),
          _vsx(0.),
          _vsy(0.),
          _My(0.),
          _engTor(0.) {}

    __device__ __host__ TMeasyNrState(double omega,
                                      double dOmega,
                                      double xt,
                                      double rStat,
                                      double fx,
                                      double fy,
                                      double fz,
                                      double vsx,
                                      double vsy,
                                      double My,
                                      double engTor)
        : _omega(omega),
          _dOmega(dOmega),
          _xt(xt),
          _rStat(rStat),
          _fx(fx),
          _fy(fy),
          _fz(fz),
          _vsx(vsx),
          _vsy(vsy),
          _My(My),
          _engTor(engTor) {}

    // Copy constructor
    __device__ __host__ TMeasyNrState(const TMeasyNrState* other)
        : _omega(other->_omega),
          _dOmega(other->_dOmega),
          _xt(other->_xt),
          _rStat(other->_rStat),
          _fx(other->_fx),
          _fy(other->_fy),
          _fz(other->_fz),
          _vsx(other->_vsx),
          _vsy(other->_vsy),
          _My(other->_My),
          _engTor(other->_engTor) {}

    // Wheel states that are set as tire states
    double _omega;   // angular velocity of wheel
    double _dOmega;  // angular velocity dot

    // Other "states" that we need to keep track of
    double _xt;            // vertical tire compression
    double _rStat;         // loaded tire radius
    double _fx, _fy, _fz;  // long, lateral and vertical force in tire frame

    // Velocities in the tire frame
    double _vsx, _vsy;

    double _My;  // Rolling resistance moment (negetive)

    // Torque from engine that we keep track of
    double _engTor;
};

// vehicle Parameters structure
struct VehicleParam {
    // default constructor with pre tuned values from HMMVW calibration
    __device__ __host__ VehicleParam()
        : _a(1.6889),
          _b(1.6889),
          _h(0.713),
          _m(2097.85),
          _jz(4519.),
          _muf(127.866),
          _mur(129.98),
          _nonLinearSteer(false),
          _maxSteer(0.6525249),
          _crankInertia(1.1),
          _tcbool(false),
          _maxBrakeTorque(4000.),
          _throttleMod(0),
          _driveType(1),
          _whichWheels(1) {}

    // Copy constructor
    __device__ __host__ VehicleParam(const VehicleParam* other)
        : _a(other->_a),
          _b(other->_b),
          _h(other->_h),
          _m(other->_m),
          _jz(other->_jz),
          _muf(other->_muf),
          _mur(other->_mur),
          _nonLinearSteer(other->_nonLinearSteer),
          _maxSteer(other->_maxSteer),
          _crankInertia(other->_crankInertia),
          _tcbool(other->_tcbool),
          _maxBrakeTorque(other->_maxBrakeTorque),
          _throttleMod(other->_throttleMod),
          _driveType(other->_driveType),
          _whichWheels(other->_whichWheels),
          _steerMap(other->_steerMap),
          _gearRatios(other->_gearRatios),
          _powertrainMap(other->_powertrainMap),
          _lossesMap(other->_lossesMap),
          _CFmap(other->_CFmap),
          _TRmap(other->_TRmap),
          _shiftMap(other->_shiftMap) {}

    double _a, _b;      // distance c.g. - front axle & distance c.g. - rear axle (m)
    double _h;          // height of c.g
    double _m;          // total vehicle mass (kg)
    double _jz;         // yaw moment inertia (kg.m^2)
    double _muf, _mur;  // front and rear unsprung mass

    // Bool that checks if the steering is non linea ---> Need to depricate
    // this, can always define a steer map 1 -> Steering is non linear, requires
    // a steering map defined - example in json 0 -> Steering is linear. Need
    // only a max steer defined. The normalized sterring then just multiplies
    // against this value to give a wheel angle
    bool _nonLinearSteer;
    // Non linear steering map in case the steering mechanism is not linear
    MapEntry* _steerMap;
    int _steerMapSize;
    // max steer angle parameters of the vehicle
    double _maxSteer;

    // crank shaft inertia
    double _crankInertia;
    // some gear parameters
    double* _gearRatios;  // gear ratio's
    int _noGears;         // number of gears

    // Additionally we have a shift map that needs to always be filled,
    // This can either be filled by setting the same upshift and downshift RPM
    // for all the gears or by setting upshift and downshift RPM for each of the
    // gears - please see the demo JSON file for an example
    MapEntry* _shiftMap;

    // boolean for torque converter presense
    bool _tcbool;

    double _maxBrakeTorque;  // max brake torque

    // Bool that defines how the throttle modulates the maps
    // 1 -> Modulates like in a motor -> Modifies the entire torque and RPM map
    // 0 -> Modulates like in an engine -> multiplies only against the torque -
    // > Default
    bool _throttleMod;
    // We will always define the powertrain with a map
    MapEntry* _powertrainMap;
    int _powertrainMapSize;
    MapEntry* _lossesMap;
    int _lossesMapSize;

    // torque converter maps
    MapEntry* _CFmap;  // capacity factor map
    int _CFmapSize;
    MapEntry* _TRmap;  // Torque ratio map
    int _TRmapSize;
    // Flag for wether we have an 2WD or 1WD
    // 1-> 2WD - This is the default
    // 0 -> 1WD - To set this, the JSON entry needs to be added.
    bool _driveType;

    // If we have a 1WD vehicle this bool specifies wether the front or the rear wheel is driven
    // powered 1 -> Rear wheel is powered 0 -> Front wheel is powered
    bool _whichWheels;
};

// vehicle states structure
struct VehicleState {
    // default constructor just assigns zero to all members
    __device__ __host__ VehicleState()
        : _x(0.),
          _y(0.),
          _dx(0),
          _dy(0),
          _u(0.),
          _v(0.),
          _psi(0.),
          _wz(0.),
          _udot(0.),
          _vdot(0.),
          _wzdot(0.),
          _tor(0.),
          _crankOmega(0.),
          _dOmega_crank(0.),
          _current_gr(0) {}

    __device__ __host__ VehicleState(double x,
                                     double y,
                                     double dx,
                                     double dy,
                                     double u,
                                     double v,
                                     double psi,
                                     double wz,
                                     double udot,
                                     double vdot,
                                     double wzdot,
                                     double tor,
                                     double crankOmega,
                                     double dOmega_crank,
                                     int current_gr)
        : _x(x),
          _y(y),
          _dx(dx),
          _dy(dy),
          _u(u),
          _v(v),
          _psi(psi),
          _wz(wz),
          _udot(udot),
          _vdot(vdot),
          _wzdot(wzdot),
          _tor(tor),
          _crankOmega(crankOmega),
          _dOmega_crank(dOmega_crank),
          _current_gr(current_gr) {}
    // copy constructor
    __device__ __host__ VehicleState(const VehicleState* other)
        : _x(other->_x),
          _y(other->_y),
          _dx(other->_dx),
          _dy(other->_dy),
          _u(other->_u),
          _v(other->_v),
          _psi(other->_psi),
          _wz(other->_wz),
          _udot(other->_udot),
          _vdot(other->_vdot),
          _wzdot(other->_wzdot),
          _tor(other->_tor),
          _crankOmega(other->_crankOmega),
          _dOmega_crank(other->_dOmega_crank),
          _current_gr(other->_current_gr) {}

    // special constructor in case need to start simulation
    // from some other state
    double _x, _y;     // x and y position
    double _dx, _dy;   // This is basically u and v but transformed to global coordinates
    double _u, _v;     // x and y velocity
    double _psi, _wz;  // yaw angle and yaw rate

    // acceleration 'states'
    double _udot, _vdot;
    double _wzdot;

    // crank torque (used to transmit torque to tires) and crank angular
    // velocity state
    double _tor;
    double _crankOmega;
    double _dOmega_crank;
    int _current_gr;
};

// --------------------------------------------------------
struct SimData {
    SimData() : _driver_data(nullptr), _driver_data_len(0) {}

    SimData(VehicleParam veh_param, TMeasyParam tireTM_param, DriverInput* driver_data)
        : _veh_param(veh_param), _tireTM_param(tireTM_param), _driver_data(driver_data) {}

    SimData(const SimData&) = delete;             // Delete copy constructor
    SimData& operator=(const SimData&) = delete;  // Delete copy assignment operator

    // Destructor
    ~SimData() {
        cudaFree(_driver_data);  // Assumes _driver_data was allocated with new[]
    }
    VehicleParam _veh_param;
    TMeasyParam _tireTM_param;
    DriverInput* _driver_data;
    unsigned int _driver_data_len;
};

struct SimDataNr {
    SimDataNr() : _driver_data(nullptr), _driver_data_len(0) {}

    SimDataNr(VehicleParam veh_param, TMeasyNrParam tireTMNr_param, DriverInput* driver_data)
        : _veh_param(veh_param), _tireTMNr_param(tireTMNr_param), _driver_data(driver_data) {}

    SimDataNr(const SimDataNr&) = delete;             // Delete copy constructor
    SimDataNr& operator=(const SimDataNr&) = delete;  // Delete copy assignment operator

    // Destructor
    ~SimDataNr() {
        cudaFree(_driver_data);  // Assumes _driver_data was allocated with new[]
    }
    VehicleParam _veh_param;
    TMeasyNrParam _tireTMNr_param;
    DriverInput* _driver_data;
    unsigned int _driver_data_len;
};

// -------------------------------------------------------------------
struct SimState {
    SimState() {}

    SimState(VehicleState veh_state, TMeasyState tiref_state, TMeasyState tirer_state)
        : _veh_state(veh_state), _tiref_state(tiref_state), _tirer_state(tirer_state) {}

    VehicleState _veh_state;
    TMeasyState _tiref_state;
    TMeasyState _tirer_state;
};

struct SimStateNr {
    SimStateNr() {}

    SimStateNr(VehicleState veh_state, TMeasyNrState tiref_state, TMeasyNrState tirer_state)
        : _veh_state(veh_state), _tiref_state(tiref_state), _tirer_state(tirer_state) {}

    VehicleState _veh_state;
    TMeasyNrState _tiref_state;
    TMeasyNrState _tirer_state;
};
// ------------------------------ Vehicle functions

__device__ double driveTorque(const VehicleParam* v_params, const double throttle, const double omega);

__device__ inline double brakeTorque(const VehicleParam* v_params, const double brake) {
    return v_params->_maxBrakeTorque * brake;
}
__device__ void differentialSplit(double torque,
                                  double max_bias,
                                  double speed_rear,
                                  double speed_front,
                                  double* torque_rear,
                                  double* torque_front,
                                  bool split,
                                  bool whichWheels);

// setting vehicle parameters using a JSON file
__host__ void setVehParamsJSON(VehicleParam& v_params, const char* fileName);

__device__ void vehToTireTransform(TMeasyState* tiref_st,
                                   TMeasyState* tirer_st,
                                   const VehicleState* v_states,
                                   const double* loads,
                                   const VehicleParam* v_params,
                                   double steering);

__device__ void vehToTireTransform(TMeasyNrState* tiref_st,
                                   TMeasyNrState* tirer_st,
                                   const VehicleState* v_states,
                                   const double* loads,
                                   const VehicleParam* v_params,
                                   double steering);

__device__ void tireToVehTransform(TMeasyState* tiref_st,
                                   TMeasyState* tirer_st,
                                   const VehicleState* v_states,
                                   const VehicleParam* v_params,
                                   double steering);

__device__ void tireToVehTransform(TMeasyNrState* tiref_st,
                                   TMeasyNrState* tirer_st,
                                   const VehicleState* v_states,
                                   const VehicleParam* v_params,
                                   double steering);

// ------------------------------ Tire functions

// sets the vertical tire deflection based on the vehicle weight
// template based on which tire
__device__ __host__ void tireInit(TMeasyParam* t_params);
__device__ __host__ void tireInit(TMeasyNrParam* t_params);

// function to calculate the force from the force charactristics
// used by tireSync
__device__ void tmxy_combined(double* f, double* fos, double s, double df0, double sm, double fm, double ss, double fs);
// Force function required by the TMeasy tire with no relaxation
__device__ void
computeCombinedColumbForce(double* fx, double* fy, double mu, double vsx, double vsy, double fz, double vcoulomb);
// setting tire parameters using a JSON file
__host__ void setTireParamsJSON(TMeasyParam& t_params, const char* fileName);
__host__ void setTireParamsJSON(TMeasyNrParam& t_params, const char* fileName);

// Functions used in the RHS function for the external solver
__device__ void computeTireLoads(double* loads,
                                 const VehicleState* v_states,
                                 const VehicleParam* v_params,
                                 const TMeasyParam* t_params);
__device__ void computeTireLoads(double* loads,
                                 const VehicleState* v_states,
                                 const VehicleParam* v_params,
                                 const TMeasyNrParam* t_params);

__device__ void computeTireRHS(TMeasyState* t_states,
                               const TMeasyParam* t_params,
                               const VehicleParam* v_params,
                               double steering);
__device__ void computeTireRHS(TMeasyNrState* t_states,
                               const TMeasyNrParam* t_params,
                               const VehicleParam* v_params,
                               double steering);
__device__ void computePowertrainRHS(VehicleState* v_states,
                                     TMeasyState* tiref_st,
                                     TMeasyState* tirer_st,
                                     const VehicleParam* v_params,
                                     const TMeasyParam* t_params,
                                     const DriverInput* controls);
__device__ void computePowertrainRHS(VehicleState* v_states,
                                     TMeasyNrState* tiref_st,
                                     TMeasyNrState* tirer_st,
                                     const VehicleParam* v_params,
                                     const TMeasyNrParam* t_params,
                                     const DriverInput* controls);
__device__ void computeVehRHS(VehicleState* v_states, const VehicleParam* v_params, const double* fx, const double* fy);

__host__ double GetTireMaxLoad(unsigned int li);
// Functions to guess tire parameters from general tire specs
__host__ void GuessTruck80Par(unsigned int li,
                              double tireWidth,
                              double ratio,
                              double rimDia,
                              double pinfl_li,
                              double pinfl_use,
                              TMeasyNrParam& t_params);

__host__ void GuessTruck80Par(double tireLoad,
                              double tireWidth,
                              double ratio,
                              double rimDia,
                              double pinfl_li,
                              double pinfl_use,
                              TMeasyNrParam& t_params);

__host__ void GuessPassCar70Par(unsigned int li,
                                double tireWidth,
                                double ratio,
                                double rimDia,
                                double pinfl_li,
                                double pinfl_use,
                                TMeasyNrParam& t_params);

__host__ void GuessPassCar70Par(double tireLoad,
                                double tireWidth,
                                double ratio,
                                double rimDia,
                                double pinfl_li,
                                double pinfl_use,
                                TMeasyNrParam& t_params);

}  // namespace d11GPU

#endif
