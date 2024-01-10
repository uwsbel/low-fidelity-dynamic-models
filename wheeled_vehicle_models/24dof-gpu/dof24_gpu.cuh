#ifndef DOF24_GPU_CUH
#define DOF24_GPU_CUH

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "utils_gpu.cuh"
// enum to decide what type of tire we have
enum class TireType { TMeasy, TMeasyNr };
namespace d24GPU {

// -----------------------------------------------------------------------------
// Tire Structs
// -----------------------------------------------------------------------------

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

    // copy constructor
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
    double _dx, _dy;       // longitudinal and lateral damping coeffs. No vertical damping

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
          _dxt(0.),
          _rStat(0.),
          _fx(0.),
          _fy(0.),
          _fz(0.),
          _fxg(0.),
          _fyg(0.),
          _fzg(0.),
          _vgx(0.),
          _vgy(0.),
          _vgz(0.),
          _vsx(0.),
          _vsy(0.),
          _vsz(0.),
          _My(0.),
          _engTor(0.) {}

    // Copy constructor
    __device__ __host__ TMeasyState(const TMeasyState* other)
        : _xe(other->_xe),
          _ye(other->_ye),
          _xedot(other->_xedot),
          _yedot(other->_yedot),
          _omega(other->_omega),
          _dOmega(other->_dOmega),
          _xt(other->_xt),
          _dxt(other->_dxt),
          _rStat(other->_rStat),
          _fx(other->_fx),
          _fy(other->_fy),
          _fz(other->_fz),
          _fxg(other->_fxg),
          _fyg(other->_fyg),
          _fzg(other->_fzg),
          _vgx(other->_vgx),
          _vgy(other->_vgy),
          _vgz(other->_vgz),
          _vsx(other->_vsx),
          _vsy(other->_vsy),
          _vsz(other->_vsz),
          _My(other->_My),
          _engTor(other->_engTor) {}

    // States that are integrated
    double _xe, _ye;        // long and lat tire deflection
    double _xedot, _yedot;  // long and lat tire deflection velocity
    double _omega;          // angular velocity of wheel
    double _dOmega;         // angular velocity dot
    double _xt;             // vertical tire compression
    double _dxt;            // vertical tire compression velocity

    // States that we keep track off
    double _rStat;  // loaded tire radius
    // long, lateral and vertical force in tire contact patch frame/ tire frame (after applying steer angle)
    double _fx, _fy, _fz;
    double _fxg, _fyg, _fzg;  // long, lateral and vertical force in global frame

    // velocites in the tire frame (transformed to tire contact path using steer angle)
    // These are obtained from the bond diagram (defines how the vehicle velocities are transformed to tire frame)
    double _vgx, _vgy, _vgz;
    // velocites in the tire contact patch frame - used in slip calculations
    double _vsx, _vsy, _vsz;

    double _My;  // Rolling resistance moment (negetive)

    // torque from engine that we keep track of
    double _engTor;
};

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
          _dxt(0.),
          _rStat(0.),
          _fx(0.),
          _fy(0.),
          _fz(0.),
          _fxg(0.),
          _fyg(0.),
          _fzg(0.),
          _vgx(0.),
          _vgy(0.),
          _vgz(0.),
          _vsx(0.),
          _vsy(0.),
          _vsz(0.),
          _My(0.),
          _engTor(0.) {}

    __device__ __host__ TMeasyNrState(double omega,
                                      double dOmega,
                                      double xt,
                                      double dxt,
                                      double rStat,
                                      double fx,
                                      double fy,
                                      double fz,
                                      double fxg,
                                      double fyg,
                                      double fzg,
                                      double vgx,
                                      double vgy,
                                      double vgz,
                                      double vsx,
                                      double vsy,
                                      double vsz,
                                      double My,
                                      double engTor)
        : _omega(omega),
          _dOmega(dOmega),
          _xt(xt),
          _dxt(dxt),
          _rStat(rStat),
          _fx(fx),
          _fy(fy),
          _fz(fz),
          _fxg(fxg),
          _fyg(fyg),
          _fzg(fzg),
          _vgx(vgx),
          _vgy(vgy),
          _vgz(vgz),
          _vsx(vsx),
          _vsy(vsy),
          _vsz(vsz),
          _My(My),
          _engTor(engTor) {}

    // Copy constructor
    __device__ __host__ TMeasyNrState(const TMeasyNrState* other)
        : _omega(other->_omega),
          _dOmega(other->_dOmega),
          _xt(other->_xt),
          _dxt(other->_dxt),
          _rStat(other->_rStat),
          _fx(other->_fx),
          _fy(other->_fy),
          _fz(other->_fz),
          _fxg(other->_fxg),
          _fyg(other->_fyg),
          _fzg(other->_fzg),
          _vgx(other->_vgx),
          _vgy(other->_vgy),
          _vgz(other->_vgz),
          _vsx(other->_vsx),
          _vsy(other->_vsy),
          _vsz(other->_vsz),
          _My(other->_My),
          _engTor(other->_engTor) {}

    // Sates that are integrated - these are actually wheel states but we don't use that abstraction
    double _omega;   // angular velocity of wheel
    double _dOmega;  // angular velocity dot
    double _xt;      // vertical tire compression
    double _dxt;     // vertical tire compression velocity

    // States that we keep track off
    double _rStat;  // loaded tire radius
    // long, lateral and vertical force in tire contact patch frame/ tire frame (after applying steer angle)
    double _fx, _fy, _fz;
    double _fxg, _fyg, _fzg;  // long, lateral and vertical force in global frame

    // velocites in the tire frame (transformed to tire contact path using steer angle)
    // These are obtained from the bond diagram (defines how the vehicle velocities are transformed to tire frame)
    double _vgx, _vgy, _vgz;
    // velocites in the tire contact patch frame - used in slip calculations
    double _vsx, _vsy, _vsz;

    double _My;  // Rolling resistance moment (negetive)

    // Torque from engine that we keep track of
    double _engTor;
};

// -----------------------------------------------------------------------------
// Vehicle Structs
// -----------------------------------------------------------------------------

struct VehicleParam {
    // default constructor
    __device__ __host__ VehicleParam()
        : _a(1.6889),
          _b(1.6889),
          _h(0.713),
          _m(2097.85),
          _jz(4519.),
          _jx(1289.),
          _jy(3516),
          _cf(1.82),
          _cr(1.82),
          _muf(127.866),
          _mur(129.98),
          _hrcf(0.379),
          _hrcr(0.327),
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
          _jx(other->_jx),
          _jy(other->_jy),
          _cf(other->_cf),
          _cr(other->_cr),
          _muf(other->_muf),
          _mur(other->_mur),
          _hrcf(other->_hrcf),
          _hrcr(other->_hrcr),
          _nonLinearSteer(other->_nonLinearSteer),
          _maxSteer(other->_maxSteer),
          _crankInertia(other->_crankInertia),
          _tcbool(other->_tcbool),
          _maxBrakeTorque(other->_maxBrakeTorque),
          _throttleMod(other->_throttleMod),
          _driveType(other->_driveType),
          _whichWheels(other->_whichWheels),
          _steerMap(other->_steerMap),
          _steerMapSize(other->_steerMapSize),
          _gearRatios(other->_gearRatios),
          _noGears(other->_noGears),
          _powertrainMap(other->_powertrainMap),
          _powertrainMapSize(other->_powertrainMapSize),
          _lossesMap(other->_lossesMap),
          _lossesMapSize(other->_lossesMapSize),
          _CFmap(other->_CFmap),
          _CFmapSize(other->_CFmapSize),
          _TRmap(other->_TRmap),
          _TRmapSize(other->_TRmapSize),
          _shiftMap(other->_shiftMap) {}

    // Declaration of all the 14 DOF vehicle parameters pretty much the same as the 8DOF model
    double _a, _b;        // distance c.g. - front axle & distance c.g. - rear axle (m)
    double _h;            // height of c.g
    double _m;            // total vehicle mass (kg)
    double _jz;           // yaw moment inertia (kg.m^2)
    double _jx;           // roll inertia
    double _jy;           // pitch inertia
    double _cf, _cr;      // front and rear track width
    double _muf, _mur;    // front and rear unsprung mass
    double _hrcf, _hrcr;  // front and rear roll centre height below C.G

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

    // double _maxTorque; // Max torque
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

    // Flag for wether we have an 4WD or 2WD
    // 1-> 4WD - This is the default
    // 0 -> 2WD - To set this, the JSON entry needs to be added.
    bool _driveType;

    // If we have a 2WD vehicle this bool specifies which of the 2 wheels are
    // powered 1 -> Rear wheels are powered 0 -> Front wheels are powered
    bool _whichWheels;
};

// Vehicle state structure for the 14dof model. Here we migth have alot of new values
struct VehicleState {
    // default constructor
    __device__ __host__ VehicleState()
        : _x(0.),
          _y(0.),
          _z(0.),
          _dx(0.),
          _dy(0.),
          _u(0.),
          _v(0.),
          _w(0.),
          _psi(0.),
          _wz(0.),
          _phi(0.),
          _wx(0.),
          _theta(0.),
          _wy(0.),
          _udot(0.),
          _vdot(0.),
          _wdot(0.),
          _wxdot(0.),
          _wydot(0.),
          _wzdot(0.),
          _dtheta(0.),
          _dphi(0.),
          _dpsi(0.),
          _tor(0.),
          _crankOmega(0.),
          _dOmega_crank(0.),
          _current_gr(0) {}
    __device__ __host__ VehicleState(const VehicleState* other)
        : _x(other->_x),
          _y(other->_y),
          _dx(other->_dx),
          _dy(other->_dy),
          _z(other->_z),
          _u(other->_u),
          _v(other->_v),
          _w(other->_w),
          _psi(other->_psi),
          _wz(other->_wz),
          _phi(other->_phi),
          _wx(other->_wx),
          _theta(other->_theta),
          _wy(other->_wy),
          _udot(other->_udot),
          _vdot(other->_vdot),
          _wdot(other->_wdot),
          _wxdot(other->_wxdot),
          _wydot(other->_wydot),
          _wzdot(other->_wzdot),
          _dtheta(other->_dtheta),
          _dphi(other->_dphi),
          _dpsi(other->_dpsi),
          _tor(other->_tor),
          _crankOmega(other->_crankOmega),
          _dOmega_crank(other->_dOmega_crank),
          _current_gr(other->_current_gr) {}
    // States
    double _x, _y, _z;  // Global x, y and z posiiton
    double _dx, _dy;
    double _u, _v, _w;   // Chassis velocities in G-RF
    double _psi, _wz;    // Yaw angle and yaw rate
    double _phi, _wx;    // roll angle and roll rate
    double _theta, _wy;  // Pitch angle and pitch rate

    // acceleration states
    double _udot, _vdot, _wdot;     // All three directions
    double _wxdot, _wydot, _wzdot;  // All three rotation directions
    double _dtheta, _dphi, _dpsi;   // Equations that we integrate for the cardan angles

    // Powertrain states
    double _tor;
    double _crankOmega;
    double _dOmega_crank;
    int _current_gr;
};

// -----------------------------------------------------------------------------
// Suspension Structs
// -----------------------------------------------------------------------------

// Suspension parameters Only 2 parameters for now, but can be easily expanded
struct SuspensionParam {
    // default constructor
    __device__ __host__ SuspensionParam() : _ks(150062), _bs(19068) {}

    // Copy Constructor
    __device__ __host__ SuspensionParam(const SuspensionParam* other) : _ks(other->_ks), _bs(other->_bs) {}
    // Parameters - Maybe later we can add map functionality
    double _ks;  // suspension stiffness
    double _bs;  // suspension damping coefficient
};

// States is why this is even a seperate struct
struct SuspensionState {
    // default constructor
    __device__ __host__ SuspensionState()
        : _xs(0.),
          _xsi(0.),
          _dxs(0.),
          _ls(0.),
          _lsi(0.),
          _us(0.),
          _vs(0.),
          _ws(0.),
          _dus(0.),
          _dvs(0.),
          _dws(0.),
          _uu(0.),
          _vu(0.),
          _wu(0.),
          _duu(0.),
          _dvu(0.),
          _dwu(0.),
          _fxs(0.),
          _fys(0.),
          _fzs(0.),
          _fzd(0.),
          _mx(0.),
          _my(0.),
          _mz(0.) {}

    // Copy constructor
    __device__ __host__ SuspensionState(const SuspensionState* other)
        : _xs(other->_xs),
          _xsi(other->_xsi),
          _dxs(other->_dxs),
          _ls(other->_ls),
          _lsi(other->_lsi),
          _us(other->_us),
          _vs(other->_vs),
          _ws(other->_ws),
          _dus(other->_dus),
          _dvs(other->_dvs),
          _dws(other->_dws),
          _uu(other->_uu),
          _vu(other->_vu),
          _wu(other->_wu),
          _duu(other->_duu),
          _dvu(other->_dvu),
          _dwu(other->_dwu),
          _fxs(other->_fxs),
          _fys(other->_fys),
          _fzs(other->_fzs),
          _fzd(other->_fzd),
          _mx(other->_mx),
          _my(other->_my),
          _mz(other->_mz) {}

    // States
    double _xs, _xsi;  // spring compression and initial spring compression
    double _dxs;       // struct compression accelerations
    double _ls, _lsi;  // struct length and initial struct length

    double _us, _vs, _ws;     // Struct velocities in three directions (G-RF)
    double _dus, _dvs, _dws;  // Struct accelerations in the three directions (G-RF)
    double _uu, _vu, _wu;     // Unsprung velocity velocities (G-RF)
    double _duu, _dvu, _dwu;  // Unsprung  accelerations (G-RF)

    // Forces and moments
    double _fxs, _fys, _fzs;  // Forces transferred to sprung mass (G-RF)
    double _fzd;              // Link load transfer forces in the z directions (jacking forces)
    double _mx, _my, _mz;     // Moments transferred to sprung mass (G-RF)
};

// -----------------------------------------------------------------------------
// Data structures that help handing multiple vehicles as needed in GPU version
// -----------------------------------------------------------------------------
// Store the parameters of the vehicle, tire and suspension needed in kernel
struct SimData {
    SimData() : _driver_data(nullptr), _driver_data_len(0) {}

    SimData(VehicleParam veh_params, TMeasyParam tireTM_params, SuspensionParam sus_params, DriverInput* driver_data)
        : _veh_params(veh_params), _tireTM_params(tireTM_params), _sus_params(sus_params), _driver_data(driver_data) {}

    SimData(const SimData&) = delete;             // Delete copy constructor
    SimData& operator=(const SimData&) = delete;  // Delete copy assignment operator

    // Destructor
    ~SimData() {
        cudaFree(_driver_data);  // Assumes _driver_data was allocated with new[]
    }
    VehicleParam _veh_params;
    TMeasyParam _tireTM_params;
    SuspensionParam _sus_params;
    DriverInput* _driver_data;
    unsigned int _driver_data_len;
};
// Same but for the TmeasyNr tire
struct SimDataNr {
    SimDataNr() : _driver_data(nullptr), _driver_data_len(0) {}

    SimDataNr(VehicleParam veh_params,
              TMeasyNrParam tireTMNr_params,
              SuspensionParam sus_params,
              DriverInput* driver_data)
        : _veh_params(veh_params),
          _tireTMNr_params(tireTMNr_params),
          _sus_params(sus_params),
          _driver_data(driver_data) {}

    SimDataNr(const SimDataNr&) = delete;             // Delete copy constructor
    SimDataNr& operator=(const SimDataNr&) = delete;  // Delete copy assignment operator

    // Destructor
    ~SimDataNr() {
        cudaFree(_driver_data);  // Assumes _driver_data was allocated with new[]
    }
    VehicleParam _veh_params;
    TMeasyNrParam _tireTMNr_params;
    SuspensionParam _sus_params;
    DriverInput* _driver_data;
    unsigned int _driver_data_len;
};

// Store the states of the vehicle, tire and suspension needed in kernel
struct SimState {
    SimState() {}

    SimState(VehicleState v_states,
             TMeasyState tirelf_st,
             TMeasyState tirerf_st,
             TMeasyState tirelr_st,
             TMeasyState tirerr_st,
             SuspensionState suslf_st,
             SuspensionState susrf_st,
             SuspensionState suslr_st,
             SuspensionState susrr_st)
        : _v_states(v_states),
          _tirelf_st(tirelf_st),
          _tirerf_st(tirerf_st),
          _tirelr_st(tirelr_st),
          _tirerr_st(tirerr_st),
          _suslf_st(suslf_st),
          _susrf_st(susrf_st),
          _suslr_st(suslr_st),
          _susrr_st(susrr_st) {}

    VehicleState _v_states;
    TMeasyState _tirelf_st;
    TMeasyState _tirerf_st;
    TMeasyState _tirelr_st;
    TMeasyState _tirerr_st;
    SuspensionState _suslf_st;
    SuspensionState _susrf_st;
    SuspensionState _suslr_st;
    SuspensionState _susrr_st;
};
// Same but for the TMeasyNr tire
struct SimStateNr {
    SimStateNr() {}

    SimStateNr(VehicleState v_states,
               TMeasyNrState tirelf_st,
               TMeasyNrState tirerf_st,
               TMeasyNrState tirelr_st,
               TMeasyNrState tirerr_st,
               SuspensionState suslf_st,
               SuspensionState susrf_st,
               SuspensionState suslr_st,
               SuspensionState susrr_st)
        : _v_states(v_states),
          _tirelf_st(tirelf_st),
          _tirerf_st(tirerf_st),
          _tirelr_st(tirelr_st),
          _tirerr_st(tirerr_st),
          _suslf_st(suslf_st),
          _susrf_st(susrf_st),
          _suslr_st(suslr_st),
          _susrr_st(susrr_st) {}

    VehicleState _v_states;
    TMeasyNrState _tirelf_st;
    TMeasyNrState _tirerf_st;
    TMeasyNrState _tirelr_st;
    TMeasyNrState _tirerr_st;
    SuspensionState _suslf_st;
    SuspensionState _susrf_st;
    SuspensionState _suslr_st;
    SuspensionState _susrr_st;
};

// -----------------------------------------------------------------------------
// Initalize functions
// -----------------------------------------------------------------------------
__host__ void tireInit(TMeasyParam* t_params);
__host__ void tireInit(TMeasyNrParam* t_params);

__host__ void initializeTireSus(const VehicleState* v_states,
                                TMeasyState* tirelf_st,
                                TMeasyState* tirerf_st,
                                TMeasyState* tirelr_st,
                                TMeasyState* tirerr_st,
                                SuspensionState* suslf_st,
                                SuspensionState* susrf_st,
                                SuspensionState* suslr_st,
                                SuspensionState* susrr_st,
                                const VehicleParam* v_params,
                                const TMeasyParam* t_params,
                                const SuspensionParam* sus_params);

__host__ void initializeTireSus(const VehicleState* v_states,
                                TMeasyNrState* tirelf_st,
                                TMeasyNrState* tirerf_st,
                                TMeasyNrState* tirelr_st,
                                TMeasyNrState* tirerr_st,
                                SuspensionState* suslf_st,
                                SuspensionState* susrf_st,
                                SuspensionState* suslr_st,
                                SuspensionState* susrr_st,
                                const VehicleParam* v_params,
                                const TMeasyNrParam* t_params,
                                const SuspensionParam* sus_params);
// -----------------------------------------------------------------------------
// Frame Transform functions
// -----------------------------------------------------------------------------
// Transform all vehicle velocites and acclerations to struct and unsprung mass velocites
__device__ void vehToSusTransform(const VehicleState* v_states,
                                  const TMeasyState* tirelf_st,
                                  const TMeasyState* tirerf_st,
                                  const TMeasyState* tirelr_st,
                                  const TMeasyState* tirerr_st,
                                  SuspensionState* suslf_st,
                                  SuspensionState* susrf_st,
                                  SuspensionState* suslr_st,
                                  SuspensionState* susrr_st,
                                  const VehicleParam* v_params,
                                  double steering);

__device__ void vehToSusTransform(const VehicleState* v_states,
                                  const TMeasyNrState* tirelf_st,
                                  const TMeasyNrState* tirerf_st,
                                  const TMeasyNrState* tirelr_st,
                                  const TMeasyNrState* tirerr_st,
                                  SuspensionState* suslf_st,
                                  SuspensionState* susrf_st,
                                  SuspensionState* suslr_st,
                                  SuspensionState* susrr_st,
                                  const VehicleParam* v_params,
                                  double steering);
// Transform all struct and unsprung mass velocites tire velocities
__device__ void vehToTireTransform(const VehicleState* v_states,
                                   TMeasyState* tirelf_st,
                                   TMeasyState* tirerf_st,
                                   TMeasyState* tirelr_st,
                                   TMeasyState* tirerr_st,
                                   const SuspensionState* suslf_st,
                                   const SuspensionState* susrf_st,
                                   const SuspensionState* suslr_st,
                                   const SuspensionState* susrr_st,
                                   const VehicleParam* v_params,
                                   double steering);

__device__ void vehToTireTransform(const VehicleState* v_states,
                                   TMeasyNrState* tirelf_st,
                                   TMeasyNrState* tirerf_st,
                                   TMeasyNrState* tirelr_st,
                                   TMeasyNrState* tirerr_st,
                                   const SuspensionState* suslf_st,
                                   const SuspensionState* susrf_st,
                                   const SuspensionState* suslr_st,
                                   const SuspensionState* susrr_st,
                                   const VehicleParam* v_params,
                                   double steering);
// Transform forces generated in the contact patch to forces in inertial frame (G-RF)
__device__ void tireToVehTransform(const VehicleState* v_states,
                                   TMeasyState* tirelf_st,
                                   TMeasyState* tirerf_st,
                                   TMeasyState* tirelr_st,
                                   TMeasyState* tirerr_st,
                                   const VehicleParam* v_params,
                                   double steering);

__device__ void tireToVehTransform(const VehicleState* v_states,
                                   TMeasyNrState* tirelf_st,
                                   TMeasyNrState* tirerf_st,
                                   TMeasyNrState* tirelr_st,
                                   TMeasyNrState* tirerr_st,
                                   const VehicleParam* v_params,
                                   double steering);
// Transform the forces from the tire to the struct in the inertial frame(G-RF) this then acts on the Chassis body
// and moves the vehicle
__device__ void computeForcesThroughSus(const VehicleState* v_states,
                                        const TMeasyState* tirelf_st,
                                        const TMeasyState* tirerf_st,
                                        const TMeasyState* tirelr_st,
                                        const TMeasyState* tirerr_st,
                                        SuspensionState* suslf_st,
                                        SuspensionState* susrf_st,
                                        SuspensionState* suslr_st,
                                        SuspensionState* susrr_st,
                                        const VehicleParam* v_params,
                                        const TMeasyParam* t_params,
                                        const SuspensionParam* sus_params);

__device__ void computeForcesThroughSus(const VehicleState* v_states,
                                        const TMeasyNrState* tirelf_st,
                                        const TMeasyNrState* tirerf_st,
                                        const TMeasyNrState* tirelr_st,
                                        const TMeasyNrState* tirerr_st,
                                        SuspensionState* suslf_st,
                                        SuspensionState* susrf_st,
                                        SuspensionState* suslr_st,
                                        SuspensionState* susrr_st,
                                        const VehicleParam* v_params,
                                        const TMeasyNrParam* t_params,
                                        const SuspensionParam* sus_params);
// -----------------------------------------------------------------------------
// Tire RHS functions and helper functions
// -----------------------------------------------------------------------------
__device__ void tmxy_combined(double* f, double* fos, double s, double df0, double sm, double fm, double ss, double fs);

// Force function required by the TMeasy tire with no relaxation
__device__ void
computeCombinedColumbForce(double* fx, double* fy, double mu, double vsx, double vsy, double fz, double vcoulomb);

__device__ void computeTireRHS(const VehicleState* v_states,
                               TMeasyState* t_states,
                               const VehicleParam* v_params,
                               const TMeasyParam* t_params,
                               double steering);
__device__ void computeTireRHS(const VehicleState* v_states,
                               TMeasyNrState* t_states,
                               const VehicleParam* v_params,
                               const TMeasyNrParam* t_params,
                               double steering);
// Computes the compress RHS that is to be integrated
__device__ void computeTireCompressionVelocity(const VehicleState* v_states,
                                               TMeasyState* tirelf_st,
                                               TMeasyState* tirerf_st,
                                               TMeasyState* tirelr_st,
                                               TMeasyState* tirerr_st,
                                               const SuspensionState* suslf_st,
                                               const SuspensionState* susrf_st,
                                               const SuspensionState* suslr_st,
                                               const SuspensionState* susrr_st);

__device__ void computeTireCompressionVelocity(const VehicleState* v_states,
                                               TMeasyNrState* tirelf_st,
                                               TMeasyNrState* tirerf_st,
                                               TMeasyNrState* tirelr_st,
                                               TMeasyNrState* tirerr_st,
                                               const SuspensionState* suslf_st,
                                               const SuspensionState* susrf_st,
                                               const SuspensionState* suslr_st,
                                               const SuspensionState* susrr_st);

// -----------------------------------------------------------------------------
// Suspension RHS functions and helper functions
// -----------------------------------------------------------------------------
// Computes the suspension deflection RHS that are to be integrated
__device__ void computeSusRHS(const VehicleState* v_states,
                              const TMeasyState* tirelf_st,
                              const TMeasyState* tirerf_st,
                              const TMeasyState* tirelr_st,
                              const TMeasyState* tirerr_st,
                              SuspensionState* suslf_st,
                              SuspensionState* susrf_st,
                              SuspensionState* suslr_st,
                              SuspensionState* susrr_st,
                              const VehicleParam* v_params,
                              const SuspensionParam* sus_params);

__device__ void computeSusRHS(const VehicleState* v_states,
                              const TMeasyNrState* tirelf_st,
                              const TMeasyNrState* tirerf_st,
                              const TMeasyNrState* tirelr_st,
                              const TMeasyNrState* tirerr_st,
                              SuspensionState* suslf_st,
                              SuspensionState* susrf_st,
                              SuspensionState* suslr_st,
                              SuspensionState* susrr_st,
                              const VehicleParam* v_params,
                              const SuspensionParam* sus_params);

// -----------------------------------------------------------------------------
// Powertrain RHS and helper functions
// -----------------------------------------------------------------------------
__device__ double driveTorque(const VehicleParam* v_params, const double throttle, const double omega);

__device__ inline double brakeTorque(const VehicleParam* v_params, const double brake) {
    return v_params->_maxBrakeTorque * brake;
}
__device__ void differentialSplit(double torque,
                                  double max_bias,
                                  double speed_left,
                                  double speed_right,
                                  double* torque_left,
                                  double* torque_right);
__device__ void computePowertrainRHS(VehicleState* v_states,
                                     TMeasyState* tirelf_st,
                                     TMeasyState* tirerf_st,
                                     TMeasyState* tirelr_st,
                                     TMeasyState* tirerr_st,
                                     const VehicleParam* v_params,
                                     const TMeasyParam* t_params,
                                     const DriverInput* controls);

__device__ void computePowertrainRHS(VehicleState* v_states,
                                     TMeasyNrState* tirelf_st,
                                     TMeasyNrState* tirerf_st,
                                     TMeasyNrState* tirelr_st,
                                     TMeasyNrState* tirerr_st,
                                     const VehicleParam* v_params,
                                     const TMeasyNrParam* t_params,
                                     const DriverInput* controls);

// -----------------------------------------------------------------------------
// Vehicle RHS and helper functions
// -----------------------------------------------------------------------------
__device__ void computeVehicleRHS(VehicleState* v_states,
                                  const TMeasyState* tirelf_st,
                                  const TMeasyState* tirerf_st,
                                  const TMeasyState* tirelr_st,
                                  const TMeasyState* tirerr_st,
                                  const SuspensionState* suslf_st,
                                  const SuspensionState* susrf_st,
                                  const SuspensionState* suslr_st,
                                  const SuspensionState* susrr_st,
                                  const VehicleParam* v_params,
                                  const TMeasyParam* t_params,
                                  const SuspensionParam* sus_params);

__device__ void computeVehicleRHS(VehicleState* v_states,
                                  const TMeasyNrState* tirelf_st,
                                  const TMeasyNrState* tirerf_st,
                                  const TMeasyNrState* tirelr_st,
                                  const TMeasyNrState* tirerr_st,
                                  const SuspensionState* suslf_st,
                                  const SuspensionState* susrf_st,
                                  const SuspensionState* suslr_st,
                                  const SuspensionState* susrr_st,
                                  const VehicleParam* v_params,
                                  const TMeasyNrParam* t_params,
                                  const SuspensionParam* sus_params);

// -----------------------------------------------------------------------------
// Json parsing functions
// -----------------------------------------------------------------------------

// --------
// Tire
// --------
__host__ void setTireParamsJSON(TMeasyParam& t_params, const char* fileName);
// setting tire parameters using a JSON file for the TMeasy NR tire
__host__ void setTireParamsJSON(TMeasyNrParam& t_params, const char* fileName);
// Additional tire functions to compute model required parameters from general parameters
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

// --------
// Suspension
// --------
__host__ void setSuspensionParamsJSON(SuspensionParam& sus_params, const char* fileName);
// --------
// Vehicle
// --------
__host__ void setVehParamsJSON(VehicleParam& v_params, const char* fileName);
}  // namespace d24GPU

#endif
