#ifndef DOF18_H
#define DOF18_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "utils.h"

namespace d18 {
/// @brief enum to decide what type of tire is being used. You will need to set this in case you wish to use the TMeasy
/// tire without relaxation (TMeasyNr where Nr stands for no relaxation). See the HalfImplicit and Sundials solver
/// documentation for more details
enum class TireType { TMeasy, TMeasyNr };
// -----------------------------------------------------------------------------
// Tire Structs
// -----------------------------------------------------------------------------

/// @brief This tire model is an implementation of the TMeasy tire model developed by Prof. Dr. Georg Rill.

/// The TMeasyParam struct holds all the parameters that are required to define a TMeasy tire. See here for more details
/// http://www.tmeasy.de/.
struct TMeasyParam {
    /// @brief This constructor sets default values for all the parameters in the TMeasyParam struct. The values are an
    /// okay proxy for a truck tire. It is highly recommended that you set the parameters yourself using the
    /// SetTireParamsJSON function that is available within this name space.
    TMeasyParam()
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

    /// @brief Copy constructor that takes in a pointer to another TMeasyParam struct and copies all the values into the
    /// new struct
    /// @param other TMeasyParam struct that is copied into the new struct
    TMeasyParam(const TMeasyParam& other)
        : _jw(other._jw),
          _rr(other._rr),
          _mu(other._mu),
          _r0(other._r0),
          _pn(other._pn),
          _pnmax(other._pnmax),
          _cx(other._cx),
          _cy(other._cy),
          _kt(other._kt),
          _dx(other._dx),
          _dy(other._dy),
          _rdyncoPn(other._rdyncoPn),
          _rdyncoP2n(other._rdyncoP2n),
          _fzRdynco(other._fzRdynco),
          _rdyncoCrit(other._rdyncoCrit),
          _dfx0Pn(other._dfx0Pn),
          _dfx0P2n(other._dfx0P2n),
          _fxmPn(other._fxmPn),
          _fxmP2n(other._fxmP2n),
          _fxsPn(other._fxsPn),
          _fxsP2n(other._fxsP2n),
          _sxmPn(other._sxmPn),
          _sxmP2n(other._sxmP2n),
          _sxsPn(other._sxsPn),
          _sxsP2n(other._sxsP2n),
          _dfy0Pn(other._dfy0Pn),
          _dfy0P2n(other._dfy0P2n),
          _fymPn(other._fymPn),
          _fymP2n(other._fymP2n),
          _fysPn(other._fysPn),
          _fysP2n(other._fysP2n),
          _symPn(other._symPn),
          _symP2n(other._symP2n),
          _sysPn(other._sysPn),
          _sysP2n(other._sysP2n) {}

    // basic tire parameters
    double _jw;  //!< Wheel inertia
    double _rr;  //!< Rolling resistance of tire
    double _mu;  //!< Friction constant
    double _r0;  //!< Unloaded tire radius

    // TM easy specific tire params
    double _pn;     //!< Nominal vertical force
    double _pnmax;  //!< Max vertical force
    double _cx;     //!< Longitudinal tire stiffness
    double _cy;     //!< Lateral tire stiffness
    double _kt;     //!< Vertical tire stiffness
    double _dx;     //!< Longitudinal tire damping coefficient
    double _dy;     //!< Lateral damping coeffs

    // TM easy force characteristic params
    // 2 values - one at nominal load and one at max load

    // dynamic radius weighting coefficient and a critical value for the
    // vertical force
    double _rdyncoPn;    //!< Dynamic radius weighting coefficient at nominal load
    double _rdyncoP2n;   //!< Dynamic radius weighting coefficient at max load
    double _fzRdynco;    //!< Nominal value the vertical force
    double _rdyncoCrit;  //!< Critical value for the vertical force

    // Longitudinal
    double _dfx0Pn;   //!< Initial longitudinal slopes dFx/dsx [N] at Nominal load
    double _dfx0P2n;  //!< Intial longitudinal slopes dFx/dsx [N] at max load
    double _fxmPn;    //!< Maximum longitudinal force [N] at nominal load
    double _fxmP2n;   //!< Maximum longitudinal force [N] at max load
    double _fxsPn;    //!< Longitudinal load at sliding [N] at nominal load
    double _fxsP2n;   //!< Longitudinal load at sliding [N] at max load
    double _sxmPn;    //!< Slip sx at maximum longitudinal load Fx at nominal load
    double _sxmP2n;   //!< Slip sx at maximum longitudinal load Fx at max load
    double _sxsPn;    //!< Slip sx where sliding begins at nominal load
    double _sxsP2n;   //!< Slip sx where sliding begins at max load
    // Lateral
    double _dfy0Pn;   //!< Intial lateral slopes dFx/dsx [N] at nominal load
    double _dfy0P2n;  //!< Intial lateral slopes dFx/dsx [N] at max load
    double _fymPn;    //!< Maximum lateral force [N] at nominal load
    double _fymP2n;   //!< Maximum lateral force [N] at max load
    double _fysPn;    //!< Lateral load at sliding [N] at nominal load
    double _fysP2n;   //!< Lateral load at sliding [N] at max load
    double _symPn;    //!< Slip sx at maximum lateral load Fx at nominal load
    double _symP2n;   //!< Slip sx at maximum lateral load Fx at max load
    double _sysPn;    //!< Slip sx where sliding begins at nominal load
    double _sysP2n;   //!< Slip sx where sliding begins at max load
};

/// @brief This tire model is an implementation of the TMeasy tire model developed by Prof. Dr. Georg Rill.

/// The TMeasyState struct holds all the states that are updates with time-integration of the TMeasy tire model. Apart
/// from the states that are updated by time integration, this struct also holds states that are updated without time
/// integration. Also, the "tire" and the "wheel" in these models are considered a single entity. Thus we also have the
/// angular velocity states as part of this struct. See here for more details about the tire model
/// http://www.tmeasy.de/.
struct TMeasyState {
    /// @brief Default constructor that sets all the states to zero
    TMeasyState()
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

    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
    TMeasyState(double xe,
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

    /// @brief Copy constructor that takes in a pointer to another TMeasyState struct and copies all the values into the
    /// new struct
    /// @param other TMeasyState struct that is copied into the new struct
    TMeasyState(const TMeasyState& other)
        : _xe(other._xe),
          _ye(other._ye),
          _xedot(other._xedot),
          _yedot(other._yedot),
          _omega(other._omega),
          _dOmega(other._dOmega),
          _xt(other._xt),
          _rStat(other._rStat),
          _fx(other._fx),
          _fy(other._fy),
          _fz(other._fz),
          _vsx(other._vsx),
          _vsy(other._vsy),
          _My(other._My),
          _engTor(other._engTor) {}

    // the actual state that are intgrated
    double _xe;      //!< Longitudinal tire deflection
    double _ye;      //!< Lateral tire deflection
    double _xedot;   //!< Longitudinal tire deflection velocity
    double _yedot;   //!< Lateral tire deflection velocity
    double _omega;   //!< Angular velocity of wheel
    double _dOmega;  //!< Angular acceleration of wheel

    // other "states" that we need to keep track of
    double _xt;     //!< Vertical tire compression
    double _rStat;  //!< Loaded tire radius
    double _fx;     //!< Longitudinal force in tire contact patch frame
    double _fy;     //!< Lateral force in tire contact patch frame
    double _fz;     //!< Vertical force in tire contact patch frame
    // velocities in tire frame
    double _vsx;  //!< Longitudinal velocity in tire contact patch frame
    double _vsy;  //!< Lateral velocity in tire contact patch frame

    double _My;      //!< Rolling resistance moment (negative)
    double _engTor;  //!< Torque from engine that we keep track of
};

/// @brief This tire model is an implementation of the TMeasy tire model developed by Prof. Dr. Georg Rill but without
/// relaxation.

/// The TMeasyNrParam (Nr stands for No relaxaation) struct holds all the parameters that are required to
/// define a TMeasyNr tire. The implementation is largely inspired by a similar model in Project Chrono (see code at
/// https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h).
struct TMeasyNrParam {
    /// @brief This constructor sets default values for all the parameters in the TMeasyNrParam struct. The values are
    /// an okay proxy for a truck tire. It is highly recommended that you set the parameters yourself using the
    /// SetTireParamsJSON function that is available within this name space.
    TMeasyNrParam()
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
    /// @brief Copy constructor that takes in a pointer to another TMeasyNrParam struct and copies all the values into
    /// the new struct
    /// @param other TMeasyNrParam struct that is copied into the new struct
    TMeasyNrParam(const TMeasyNrParam& other)
        : _jw(other._jw),
          _rr(other._rr),
          _mu(other._mu),
          _r0(other._r0),
          _width(other._width),
          _rim_radius(other._rim_radius),
          _pn(other._pn),
          _pnmax(other._pnmax),
          _kt(other._kt),
          _rdyncoPn(other._rdyncoPn),
          _rdyncoP2n(other._rdyncoP2n),
          _fzRdynco(other._fzRdynco),
          _rdyncoCrit(other._rdyncoCrit),
          _dfx0Pn(other._dfx0Pn),
          _dfx0P2n(other._dfx0P2n),
          _fxmPn(other._fxmPn),
          _fxmP2n(other._fxmP2n),
          _fxsPn(other._fxsPn),
          _fxsP2n(other._fxsP2n),
          _sxmPn(other._sxmPn),
          _sxmP2n(other._sxmP2n),
          _sxsPn(other._sxsPn),
          _sxsP2n(other._sxsP2n),
          _dfy0Pn(other._dfy0Pn),
          _dfy0P2n(other._dfy0P2n),
          _fymPn(other._fymPn),
          _fymP2n(other._fymP2n),
          _fysPn(other._fysPn),
          _fysP2n(other._fysP2n),
          _symPn(other._symPn),
          _symP2n(other._symP2n),
          _sysPn(other._sysPn),
          _sysP2n(other._sysP2n),
          _vcoulomb(other._vcoulomb),
          _frblend_begin(other._frblend_begin),
          _frblend_end(other._frblend_end),
          _bearingCapacity(other._bearingCapacity),
          _li(other._li),
          _p_li(other._p_li),
          _p_use(other._p_use) {}

    double _jw;  //!< Wheel inertia
    double _rr;  //!< Rolling resistance of tire
    double _mu;  //!< Friction constant
    double _r0;  //!< Unloaded tire radius

    double _width;       //!< Tire width
    double _rim_radius;  //!< Rim radius
    double _pn;          //!< Nominal vertical force
    double _pnmax;       //!< Max vertical force
    double _kt;          //!< Vertical tire stiffness
    double _rdyncoPn;    //!< Dynamic radius weighting coefficient at nominal load
    double _rdyncoP2n;   //!< Dynamic radius weighting coefficient at max load
    double _fzRdynco;    //!< Nominal value the vertical force
    double _rdyncoCrit;  //!< Critical value for the vertical force
    // Longitudinal
    double _dfx0Pn;   //!< Initial longitudinal slopes dFx/dsx [N] at Nominal load
    double _dfx0P2n;  //!< Intial longitudinal slopes dFx/dsx [N] at max load
    double _fxmPn;    //!< Maximum longitudinal force [N] at nominal load
    double _fxmP2n;   //!< Maximum longitudinal force [N] at max load
    double _fxsPn;    //!< Longitudinal load at sliding [N] at nominal load
    double _fxsP2n;   //!< Longitudinal load at sliding [N] at max load
    double _sxmPn;    //!< Slip sx at maximum longitudinal load Fx at nominal load
    double _sxmP2n;   //!< Slip sx at maximum longitudinal load Fx at max load
    double _sxsPn;    //!< Slip sx where sliding begins at nominal load
    double _sxsP2n;   //!< Slip sx where sliding begins at max load
    // Lateral
    double _dfy0Pn;    //!< Intial lateral slopes dFx/dsx [N] at nominal load
    double _dfy0P2n;   //!< Intial lateral slopes dFx/dsx [N] at max load
    double _fymPn;     //!< Maximum lateral force [N] at nominal load
    double _fymP2n;    //!< Maximum lateral force [N] at max load
    double _fysPn;     //!< Lateral load at sliding [N] at nominal load
    double _fysP2n;    //!< Lateral load at sliding [N] at max load
    double _symPn;     //!< Slip sx at maximum lateral load Fx at nominal load
    double _symP2n;    //!< Slip sx at maximum lateral load Fx at max load
    double _sysPn;     //!< Slip sx where sliding begins at nominal load
    double _sysP2n;    //!< Slip sx where sliding begins at max load
    double _vcoulomb;  //!< Velocity below which we care about static friction -> Defaults to 1 if user does not provide
                       //!< this
    double _frblend_begin;  //!< Beginning of friction blending used instead of relexation -> Defaults to 1 if the user
                            //!< does not provide this
    double _frblend_end;  //!< End of friction blending used instead of relexation -> Defaults to 1 if the user does not
                          //!< provide this
    double _bearingCapacity;  //!< In case bearing capacity of the tire is known, it can be provided. If not, the
                              //!< "loadIndex" is used to calculate the bearing capacity
    double _li;               // Load index
    double _p_li;             // Pressure at load index -> Defaults to 0 if the user does not provide this
    double _p_use;            // Pressure at which the tire is used -> Defaults to 0 if the user does not provide this
};
/// @brief This tire model is an implementation of the TMeasy tire model developed by Prof. Dr. Georg Rill but without
/// relaxation.

/// The TMeasyNrState (Nr stands for No relaxaation) struct holds all the states that are updates with
/// time-integration of the TMeasy tire model. Apart from the states that are updated by time integration, this struct
/// also holds states that are updated without time integration. Also, the "tire" and the "wheel" in these models are
/// considered a single entity. Thus we also have the angular velocity states as part of this struct. The implementation
/// is largely inspired by a similar model in Project Chrono (see code at
/// https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h).
struct TMeasyNrState {
    /// @brief Default constructor that sets all the states to zero
    TMeasyNrState()
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
    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
    TMeasyNrState(double omega,
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

    /// @brief Copy constructor that takes in a pointer to another TMeasyNrState struct and copies all the values into
    /// the new struct
    /// @param other TMeasyNrState struct that is copied into the new struct
    TMeasyNrState(const TMeasyNrState& other)
        : _omega(other._omega),
          _dOmega(other._dOmega),
          _xt(other._xt),
          _rStat(other._rStat),
          _fx(other._fx),
          _fy(other._fy),
          _fz(other._fz),
          _vsx(other._vsx),
          _vsy(other._vsy),
          _My(other._My),
          _engTor(other._engTor) {}

    // Wheel states that are set as tire states
    double _omega;   //!< Angular velocity of wheel
    double _dOmega;  //!< Angular acceleration of wheel

    // Other "states" that we need to keep track of
    double _xt;     //!< Vertical tire compression
    double _rStat;  //!< Loaded tire radius
    // long, lateral and vertical force in tire contact patch frame/ tire frame (after applying steer angle)
    double _fx;  //!< Longitudinal force in tire contact patch frame
    double _fy;  //!< Lateral force in tire contact patch frame
    double _fz;  //!< Vertical force in tire contact patch frame

    double _vsx;  //!< Longitudinal velocity in tire contact patch frame
    double _vsy;  //!< Lateral velocity in tire contact patch frame

    double _My;      //!< Rolling resistance moment (negative)
    double _engTor;  //!< Torque from engine that we keep track of
};
// -----------------------------------------------------------------------------
// Vehicle Structs
// -----------------------------------------------------------------------------
/// @brief Defined here are chassis, engine/motor, powertrain, driveline and steering parameters required for the
/// simualtion of a 18 DOF model.

/// The 18 DOF model does not capture pitch and heave motions and the front and rear suspension are represented simply
/// by their respective roll stiffness and roll damping coefficients. Additionally, wheel lift off conditions cannot be
/// simulated. The vertical forces Fzij are obtained based on quasi-static lateral and longitudinal load transfers. See
/// chapter 2 here for more details https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc.
struct VehicleParam {
    /// @brief This constructor sets default values for all the parameters in the VehicleParam struct. The values are an
    /// okay proxy for a truck (namely the HMMWV model).  It is highly recommended that you set the parameters yourself
    /// using the setVehParamsJSON function that is available within this name space.
    VehicleParam()
        : _a(1.6889),
          _b(1.6889),
          _h(0.713),
          _m(2097.85),
          _jz(4519.),
          _jx(1289.),
          _jxz(3.265),
          _cf(1.82),
          _cr(1.82),
          _muf(127.866),
          _mur(129.98),
          _hrcf(0.379),
          _hrcr(0.327),
          _krof(31000),
          _kror(31000),
          _brof(3300),
          _bror(3300),
          _nonLinearSteer(false),
          _maxSteer(0.6525249),
          _crankInertia(1.1),
          _tcbool(false),
          _maxBrakeTorque(4000.),
          _throttleMod(0),
          _driveType(1),
          _whichWheels(1) {}

    /// @brief Copy constructor that takes in a pointer to another VehicleParam struct and copies all the values into
    /// the new struct
    /// @param other VehicleParam struct that is copied into the new struct
    VehicleParam(const VehicleParam& other)
        : _a(other._a),
          _b(other._b),
          _h(other._h),
          _m(other._m),
          _jz(other._jz),
          _jx(other._jx),
          _jxz(other._jxz),
          _cf(other._cf),
          _cr(other._cr),
          _muf(other._muf),
          _mur(other._mur),
          _hrcf(other._hrcf),
          _hrcr(other._hrcr),
          _krof(other._krof),
          _kror(other._kror),
          _brof(other._brof),
          _bror(other._bror),
          _nonLinearSteer(other._nonLinearSteer),
          _maxSteer(other._maxSteer),
          _crankInertia(other._crankInertia),
          _tcbool(other._tcbool),
          _maxBrakeTorque(other._maxBrakeTorque),
          _throttleMod(other._throttleMod),
          _driveType(other._driveType),
          _whichWheels(other._whichWheels),
          _steerMap(other._steerMap),
          _gearRatios(other._gearRatios),
          _powertrainMap(other._powertrainMap),
          _lossesMap(other._lossesMap),
          _CFmap(other._CFmap),
          _TRmap(other._TRmap),
          _shiftMap(other._shiftMap) {}

    double _a;     //!< Distance of C.G to front axle (m)
    double _b;     //!< Distance of C.G to rear axle (m)
    double _h;     //!< Height of C.G
    double _m;     //!< Total vehicle mass (kg)
    double _jz;    //!< Yaw moment of inertia (kg.m^2)
    double _jx;    //!< Roll inertia (kg.m^2)
    double _jxz;   //!< XZ Product of inertia (kg.m^2)
    double _cf;    //!< Front track width (m)
    double _cr;    //!< Rear track width (m)
    double _muf;   //!< Front unsprung mass (kg)
    double _mur;   //!< Rear unsprung mass (kg)
    double _hrcf;  //!< Front roll centre height below C.G (m)
    double _hrcr;  //!< Rear roll centre height below C.G (m)
    double _krof;  //!< Front roll stiffness (N/rad)
    double _kror;  //!< Rear roll stiffness (N/rad)
    double _brof;  //!< Front roll damping (N.m.s/rad)
    double _bror;  // Rear roll damping (N.m.s/rad)

    /**
     * @brief Bool that checks if the steering is non linear. A non-linear map can be defined in a json file and read
     * using setVehParamsJSON.
     *
     * 1 -> Steering is non linear, requires a steering map defined
     * 0 -> Steering is linear. Need only a _maxSteer defined.
     */
    bool _nonLinearSteer;
    /**
     * @brief _steerMap stores the non-linear steering map (in case required by the user) and can be set using the JSON
     * file.
     *
     *  _steerMap is a vector of MapEntry's where the x component of the MapEntry is the normalized steering input
     * (between -1 and 1 where negative implies a left turn) and the y component is the wheel angle. For steerng inputs
     * between the values specified by the MapEntry's, the wheel angle is linearly interpolated. Set using the
     * "steerMap" key in the JSON file.
     */
    std::vector<MapEntry> _steerMap;

    double _maxSteer;  //!< In case _nonLinearSteer is set to 0, this is the maximum wheel angle that can be achieved
                       //!< (in radians). Values at other steering wheel inputs are then linearly interpolated.

    double _crankInertia;             //!< Inertia of the engine/motor crankshaft
    std::vector<double> _gearRatios;  //!< Vector of gear ratios
    /**
     * @brief A shift map defines the RPMs at which the various gears shift.
     *
     * _shiftMap is a vector of MapEntry's where the x component of the MapEntry is the down shift RPM and the y
     * component is the upshift RPM. The user can either specify the shift map for all the gears or can specify an
     * "upshiftRPM" and a "downshiftRPM". These two values are then applied for all the gears (see json files for
     * examples)
     */
    std::vector<MapEntry> _shiftMap;

    bool _tcbool;  //!< Boolean that checks for the presence of a torque converter. Can be set using "tcBool" in the
                   //!< JSON file. Defaults to 0 if not specified.

    double _maxBrakeTorque;  //!< The maximum braking torque (Nm) that the brakes applu to the wheels. Can be set using
                             //!< "maxBrakeTorque" in the JSON file. Based on the normalized brake input between 0 and
                             //!< 1, a torque input*_maxBrakeTorque is applied to the wheel.

    /**
     * @brief Bool that defines how the throttle modulates the Torque map of the motor
     * 1 -> Modulates like in a motor -> Modifies the entire torque and RPM map
     * 0 -> Modulates like in an engine -> multiplies only against the torque -> Default
     * Can be set using "throttleMod" in the JSON file.
     * For more details see implementation of function driveTorque.
     */
    bool _throttleMod;
    /**
     * @brief _powertrainMap stores the powertrain map.
     *
     * _powertrainMap is a vector of MapEntry's where the x component of the MapEntry is the engine crankshaft RPM
     and the y
     * component is the torque. For RPMs between the values specified by the MapEntry's, the torque is linearly
     interpolated. Set using the "torqueMap" key in the JSON file.
     */
    std::vector<MapEntry> _powertrainMap;
    /**
     * @brief _lossesMap stores the powertrain losses map and can be set using the JSON file using the "lossesMap" key.
     * _lossesMap is a vector of MapEntry's where the x component of the MapEntry is the engine crankshaft RPM and
     * the y component is the losses in Nm. For RPMs between the values specified by the MapEntry's, the losses are
     * linearly interpolated. The losses are subtracted from the torque obtained from the powertrain map after throttle
     * is applied. See function driveTorque for more details.
     * Set using the "lossesMap" key in the JSON file.
     */
    std::vector<MapEntry> _lossesMap;

    /**
     * @brief In case a torque converter is part of the model, the capacity map needs to be defined.
     *
     * _CFmap is a vector of MapEntry's where the x component of the MapEntry is the speed ratio between the
     * driven and driving shafts (whether the engine crank is driven or driving depends on if there is a throttle
     * applied) and the y component is the capacity factor. For speed ratios between the values specified by the
     * MapEntry's, the capacity factor is linearly interpolated.
     * Set using the "capacityFactorMap" key in the JSON file.
     */
    std::vector<MapEntry> _CFmap;  // capacity factor map
    /**
     * @brief In case a torque converter is part of the model, the torque ratio map needs to be defined.
     *
     * _TRmap points is a vector of MapEntry's where the x component of the MapEntry is the speed ratio between the
     * driven and driving shafts (whether the engine crank is driven or driving depends on if there is a throttle
     * applied) and the y component is the torque ratio. For speed ratios between the values specified by the
     * MapEntry's, the torque ratio is linearly interpolated. Set using the "torqueRatioMap" key in the JSON file.
     */
    std::vector<MapEntry> _TRmap;  // Torque ratio map

    /**
     * @brief This boolean specifies whether the vehicle has a 4WD or a 2WD drivetrain.
     *
     * 1 -> 4WD - This is the default
     * 0 -> 2WD - To set this, the JSON entry needs to be added.
     * This can be set using the "4wd" key in the JSON file.
     */
    bool _driveType;

    /**
     * @brief This boolean specifies whether the vehicle is front wheel drive or rear wheel drive in case its a 2WD
     *
     * 1 -> Rear wheel drive
     * 0 -> Front wheel drive
     * This can be set using the "rearWheels" key in the JSON file.
     */
    bool _whichWheels;
};

/// @brief The VehicleState struct holds the chassis, engine/motor, powertrain, driveline and steering states required
/// for the simualtion.

/// Apart from the states that are updated by time integration, this struct also holds "non-states"
///  such as accelerations, forces and torques that are not updated by time integration. See here for more details
///  https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc. important to note that this implementation within the
///  d18GPU namespace is exactly the same as the implementation in the d18 namespace. The only difference is that the
///  d18GPU namespace is meant to be used on the GPU where standard library functions are not available.
struct VehicleState {
    /// @brief Default constructor that sets all the states to zero
    VehicleState()
        : _x(0.),
          _y(0.),
          _dx(0),
          _dy(0),
          _u(0.),
          _v(0.),
          _psi(0.),
          _wz(0.),
          _phi(0.),
          _wx(0.),
          _udot(0.),
          _vdot(0.),
          _wxdot(0.),
          _wzdot(0.),
          _tor(0.),
          _crankOmega(0.),
          _current_gr(0) {}

    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
    VehicleState(double x,
                 double y,
                 double dx,
                 double dy,
                 double u,
                 double v,
                 double psi,
                 double wz,
                 double phi,
                 double wx,
                 double udot,
                 double vdot,
                 double wxdot,
                 double wzdot,
                 double tor,
                 double crankOmega,
                 int current_gr)
        : _x(x),
          _y(y),
          _dx(dx),
          _dy(dy),
          _u(u),
          _v(v),
          _psi(psi),
          _wz(wz),
          _phi(phi),
          _wx(wx),
          _udot(udot),
          _vdot(vdot),
          _wxdot(wxdot),
          _wzdot(wzdot),
          _tor(tor),
          _crankOmega(crankOmega),
          _current_gr(current_gr) {}

    /// @brief Copy constructor that takes in a pointer to another VehicleState struct and copies all the values into
    /// the new struct
    /// @param other VehicleState struct that is copied into the new struct
    VehicleState(const VehicleState& other)
        : _x(other._x),
          _y(other._y),
          _dx(other._dx),
          _dy(other._dy),
          _u(other._u),
          _v(other._v),
          _psi(other._psi),
          _wz(other._wz),
          _phi(other._phi),
          _wx(other._wx),
          _udot(other._udot),
          _vdot(other._vdot),
          _wxdot(other._wxdot),
          _wzdot(other._wzdot),
          _tor(other._tor),
          _crankOmega(other._crankOmega),
          _dOmega_crank(other._dOmega_crank),
          _current_gr(other._current_gr) {}

    // special constructor in case need to start simulation
    // from some other state
    double _x;    //!< Global x position
    double _y;    //!< Global y position
    double _dx;   //!< This is the global x velocity but is only used by the sundials solver to integrate to find x
    double _dy;   //!< This is the global y velocity but is only used by the sundials solver to integrate to find x
    double _u;    //!< Chassis x velocity in G-RF
    double _v;    //!< Chassis y velocity in G-RF
    double _psi;  //!< Yaw angle of chassis (rotation about z axis of G-RF) - this is the angle that is used to rotate
                  //!< the chassis frame to the global frame
    double _wz;   //!< Yaw rate of chassis
    double _phi;  //!< Roll angle of chassis (rotation about x axis of G-RF) - this is the angle that is used to rotate
                  //!< the chassis frame to the global frame
    double _wx;   //!< Roll rate of chassis

    // acceleration 'states'
    double _udot;   //!< Chassis x acceleration in G-RF
    double _vdot;   //!< Chassis y acceleration in G-RF
    double _wxdot;  //!< Roll acceleration of chassis
    double _wzdot;  //!< Yaw acceleration of chassis

    // Powertrain states
    double _tor;           //!< Engine/motor torque for writing to output
    double _crankOmega;    //!< Crankshaft angular velocity
    double _dOmega_crank;  //!< Crankshaft angular acceleration
    int _current_gr;       //!< Current gear
};

// -----------------------------------------------------------------------------
// Initalize functions
// -----------------------------------------------------------------------------
/// @brief Initializes the nominal and critical value for the vertical force based on tire parameters
/// @param t_params TMeasytire parameters
void tireInit(TMeasyParam& t_params);
/// @brief Overload of tireInit for TMeasyNr tire model
/// @param t_params TMeasyNr tire parameters
void tireInit(TMeasyNrParam& t_params);

// -----------------------------------------------------------------------------
// Frame Transform functions
// -----------------------------------------------------------------------------
/// @brief Transform all vehicle velocites to tire velocities
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Right Front (RF))
/// @param tirelr_st TMeasy tire states (Left Rear (LR))
/// @param tirerr_st TMeasy tire states (Right Rear (RR))
/// @param v_states Vehicle states
/// @param loads Tire vertical loads
/// @param v_params Vehicle parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
void vehToTireTransform(TMeasyState& tirelf_st,
                        TMeasyState& tirerf_st,
                        TMeasyState& tirelr_st,
                        TMeasyState& tirerr_st,
                        const VehicleState& v_states,
                        const std::vector<double>& loads,
                        const VehicleParam& v_params,
                        double steering);
/// @brief Overload of vehToTireTransform for TMeasyNr tire model
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Right Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Rear (LR))
/// @param tirerr_st TMeasyNr tire states (Right Rear (RR))
/// @param v_states Vehicle states
/// @param loads Tire vertical loads
/// @param v_params Vehicle parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
void vehToTireTransform(TMeasyNrState& tirelf_st,
                        TMeasyNrState& tirerf_st,
                        TMeasyNrState& tirelr_st,
                        TMeasyNrState& tirerr_st,
                        const VehicleState& v_states,
                        const std::vector<double>& loads,
                        const VehicleParam& v_params,
                        double steering);
/// @brief Transform forces generated in the contact patch to forces in inertial frame (G-RF)
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Right Front (RF))
/// @param tirelr_st TMeasy tire states (Left Rear (LR))
/// @param tirerr_st TMeasy tire states (Right Rear (RR))
/// @param v_states Vehicle states
/// @param v_params Vehicle parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
void tireToVehTransform(TMeasyState& tirelf_st,
                        TMeasyState& tirerf_st,
                        TMeasyState& tirelr_st,
                        TMeasyState& tirerr_st,
                        const VehicleState& v_states,
                        const VehicleParam& v_params,
                        double steering);
/// @brief Overload of tireToVehTransform for TMeasyNr tire model
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Right Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Rear (LR))
/// @param tirerr_st TMeasyNr tire states (Right Rear (RR))
/// @param v_states Vehicle states
/// @param v_params Vehicle parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
void tireToVehTransform(TMeasyNrState& tirelf_st,
                        TMeasyNrState& tirerf_st,
                        TMeasyNrState& tirelr_st,
                        TMeasyNrState& tirerr_st,
                        const VehicleState& v_states,
                        const VehicleParam& v_params,
                        double steering);

// -----------------------------------------------------------------------------
// Tire RHS functions and helper functions
// -----------------------------------------------------------------------------

/// @brief Helper function to provide the tire force (f) and tire force / combined slip (fos) for the combined slip
/// model of the TMeasy and TMeasyNr tire models.

/// See page 78 of book Road Vehicle Dynamics: Fundamentals and Modeling by Georg Rill for more details about other
/// input parameters
void tmxy_combined(double& f, double& fos, double s, double df0, double sm, double fm, double ss, double fs);

/// @brief Computes the combined columnb force for the TMeasyNr tire model

/// This force provides the stability at low speeds and is blended with the slip force provided by the tmxy_combined
/// function.
void computeCombinedCoulombForce(double& fx, double& fy, double mu, double vsx, double vsy, double fz, double vcoulomb);

/// @brief Compute the vertical loads on the tire using quasi-static load transfer equations.

/// See chapter 2 here for
/// more details here for more details https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc
/// @param loads Computed vertical loads on each tire (LF, RF, LR, RR)
/// @param v_states Vehicle states
/// @param v_params Vehicle parameters
/// @param t_params Tire parameters
void computeTireLoads(std::vector<double>& loads,
                      const VehicleState& v_states,
                      const VehicleParam& v_params,
                      const TMeasyParam& t_params);

/// @brief computeTireLoads overloaded for TMeasyNr tire model
/// @param v_states Vehicle states
/// @param v_params Vehicle parameters
/// @param t_params Tire parameters
void computeTireLoads(std::vector<double>& loads,
                      const VehicleState& v_states,
                      const VehicleParam& v_params,
                      const TMeasyNrParam& t_params);

/// @brief Computes the tire forces for the TMeasy tire model in the tire contact patch frame (T-RF)

/// The tire slip force produces a lateral and longitudinal tire deflection which produces the forces. A lot more detail
/// of the formulation can be found in the book Road Vehicle Dynamics: Fundamentals and Modeling by Georg Rill
/// @param t_states TMeasy tire states
/// @param t_params TMeasy tire parameters
/// @param v_params Vehicle Parameters
/// @param steering Steering input used to calculate the lateral tire slip
void computeTireRHS(TMeasyState& t_states, const TMeasyParam& t_params, const VehicleParam& v_params, double steering);

/// @brief Computes the tire forces for the TMeasyNr tire model in the tire contact patch frame (T-RF)

/// For the TMeasyNr tire model, since there is no relaxation, the tire forces from slip are blended with the tire
/// forces from coulomb friction. The blend coefficient depends on the longitudinal slip velocity of the tire. This model
/// is an approximation of the original TMeasy tire model and is inspired by the Project Chrono implementation (see code
/// at https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h).
/// @param t_states TMeasyNr tire states
/// @param t_params TMeasyNr tire parameter
/// @param v_params Vehicle Parameters
/// @param steering Steering input used to calculate the lateral tire slip
void computeTireRHS(TMeasyNrState& t_states,
                    const TMeasyNrParam& t_params,
                    const VehicleParam& v_params,
                    double steering);

// -----------------------------------------------------------------------------
// Powertrain RHS and helper functions
// -----------------------------------------------------------------------------
/// @brief Computes the Engine drive torque based on throttle input, engine omega _throttleMod, the _powertrainMap and
/// the _lossesMap.
/// @param v_params Vehicle Parameters
/// @param throttle Throttle input [0-1]
/// @param omega Engine angular velocity (rad/s)
double driveTorque(const VehicleParam& v_params, const double throttle, const double omega);

/// @brief Computes the Engine braking torque based on brake input and _maxBrakeTorque
/// @param v_params Vehicle Parameters
/// @param brake Brake input [0-1]
inline double brakeTorque(const VehicleParam& v_params, const double brake) {
    return v_params._maxBrakeTorque * brake;
}

/// @brief  Helper function that calculates the torque split to each tire based on the differential max bias and the
/// tires relative angular velocity
/// @param torque Torque to differential
/// @param max_bias Bias that determines the impact of relative angular velocity on torque split
/// @param speed_left Angular velocity of left tire
/// @param speed_right Angular velocity of right tire
/// @param torque_left Torque provided to the left tire
/// @param torque_right Torque provided to the right tire
void differentialSplit(double torque,
                       double max_bias,
                       double speed_left,
                       double speed_right,
                       double& torque_left,
                       double& torque_right);

/// @brief Computes the Crank-Shaft angular acceleration (if there is a torque converter) as well as the angular wheel
/// accelerations that are integrated to provide the wheel angular velocities.

/// In case there is no torque converter, the crank-shaft angular velocity is calculated solely based on the wheel
/// angular velocities (not integrated). This function is also where the velocities go from the wheel to the engine
/// (through the differential, gear box, torque converter (optional)) and the torques go from the engine to the wheel
/// (through the differential, gear box, torque converter (optional)).
/// @param v_states Vehicle states
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Right Front (RF))
/// @param tirelr_st TMeasy tire states (Left Rear (LR))
/// @param tirerr_st TMeasy tire states (Right Rear (RR))
/// @param v_params Vehicle parameters
/// @param t_params Tire parameters
/// @param controls Vehicle driver inputs (throttle, brake, steering)
void computePowertrainRHS(VehicleState& v_states,
                          TMeasyState& tirelf_st,
                          TMeasyState& tirerf_st,
                          TMeasyState& tirelr_st,
                          TMeasyState& tirerr_st,
                          const VehicleParam& v_params,
                          const TMeasyParam& t_params,
                          const DriverInput& controls);

/// @brief computePowertrainRHS overloaded for TMeasyNr tire model
/// @param v_states Vehicle states
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Right Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Rear (LR))
/// @param tirerr_st TMeasyNr tire states (Right Rear (RR))
/// @param v_params Vehicle parameters
/// @param t_params Tire parameters
/// @param controls Vehicle driver inputs (throttle, brake, steering)
void computePowertrainRHS(VehicleState& v_states,
                          TMeasyNrState& tirelf_st,
                          TMeasyNrState& tirerf_st,
                          TMeasyNrState& tirelr_st,
                          TMeasyNrState& tirerr_st,
                          const VehicleParam& v_params,
                          const TMeasyNrParam& t_params,
                          const DriverInput& controls);

// -----------------------------------------------------------------------------
// Vehicle RHS and helper functions
// -----------------------------------------------------------------------------

/// @brief Computes the vehicle linear and angular accelerations that are to be integrated
/// @param v_states Vehicle states
/// @param v_params Vehicle parameters
/// @param fx Tire longitudinal forces in inertial frame (G-RF) (LF, RF, LR, RR)
/// @param fy Tire lateral forces in inertial frame (G-RF) (LF, RF, LR, RR)
void computeVehRHS(VehicleState& v_states,
                   const VehicleParam& v_params,
                   const std::vector<double>& fx,
                   const std::vector<double>& fy);

// -----------------------------------------------------------------------------
// Json parsing functions
// -----------------------------------------------------------------------------

// --------
// Tire
// --------
/// @brief Parses the tire parameters from a JSON file for the TMeasy tire model. Please see implementation and/or
/// example json files for appropriate keys and format.
/// @param t_params TMeasy tire parameters
/// @param fileName Path to JSON file (see demo for example)
void setTireParamsJSON(TMeasyParam& t_params, const char* fileName);

/// @brief Parses the tire parameters from a JSON file for the TMeasyNr tire model. Please see implementation and/or
/// example json files for appropriate keys and format.

/// The TMeasyNr follows a different approach to setting tire parameters as it only requires high level tire parameters
/// to be set by the user. However, the user can still set the tire parameters directly if they wish to do so by setting
/// the "highLevelParams" key to false.
/// @param t_params TMeasyNr tire parameters
/// @param fileName Path to JSON file (see demo for example)
void setTireParamsJSON(TMeasyNrParam& t_params, const char* fileName);

/// @brief Helper function to compute the max tire load from the load index
/// @param li Load index
double GetTireMaxLoad(unsigned int li);

/// @brief Functions to guess tire parameters from general truck tire. The ability to do this is one of the major
/// advanatages of the TMeasy tire models.

/// This function will be called if the user sets the key "vehicleType" to "Truck" in the JSON file
/// @param li load index
/// @param tireWidth Tire width
/// @param ratio Tire aspect ration
/// @param rimDia Tire rim diameter
/// @param pinfl_li Designed inflation pressure for the load index
/// @param pinfl_use Used inflation pressure
/// @param t_params TMeasyNr tire parameters
void GuessTruck80Par(unsigned int li,
                     double tireWidth,
                     double ratio,
                     double rimDia,
                     double pinfl_li,
                     double pinfl_use,
                     TMeasyNrParam& t_params);

/// @brief Overload of GuessTruck80Par for TMeasyNr tire model when tireload is provided directly instead of load index
/// @param tireLoad Tire load
/// @param tireWidth Tire width
/// @param ratio Tire aspect ration
/// @param rimDia Tire rim diameter
/// @param pinfl_li Designed inflation pressure for the load index
/// @param pinfl_use Used inflation pressure
/// @param t_params TMeasyNr tire parameters
void GuessTruck80Par(double tireLoad,
                     double tireWidth,
                     double ratio,
                     double rimDia,
                     double pinfl_li,
                     double pinfl_use,
                     TMeasyNrParam& t_params);

/// @brief For passenger tires. This function is called internally when "vehicleType" is set to "Passenger" in the JSON
/// @param li load index
/// @param tireWidth Tire width
/// @param ratio Tire aspect ration
/// @param rimDia Tire rim diameter
/// @param pinfl_li Designed inflation pressure for the load index
/// @param pinfl_use Used inflation pressure
/// @param t_params TMeasyNr tire parameters
void GuessPassCar70Par(unsigned int li,
                       double tireWidth,
                       double ratio,
                       double rimDia,
                       double pinfl_li,
                       double pinfl_use,
                       TMeasyNrParam& t_params);

/// @brief Overload of GuessPassCar70Par for TMeasyNr tire model when tireload is provided directly instead of load
/// index
/// @param tireLoad Tire load
/// @param tireWidth Tire width
/// @param ratio Tire aspect ration
/// @param rimDia Tire rim diameter
/// @param pinfl_li Designed inflation pressure for the load index
/// @param pinfl_use Used inflation pressure
/// @param t_params TMeasyNr tire parameters
void GuessPassCar70Par(double tireLoad,
                       double tireWidth,
                       double ratio,
                       double rimDia,
                       double pinfl_li,
                       double pinfl_use,
                       TMeasyNrParam& t_params);

// --------
// Vehicle
// --------
/// @brief Parses the vehicle parameters from a JSON file for the TMeasy tire model.
/// @param v_params Vehicle parameters
/// @param fileName Path to JSON file (see demo for example)
void setVehParamsJSON(VehicleParam& v_params, const char* fileName);
}  // namespace d18

#endif
