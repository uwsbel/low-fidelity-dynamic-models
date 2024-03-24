#ifndef DOF24_GPU_CUH
#define DOF24_GPU_CUH

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "utils_gpu.cuh"
/// @brief enum to decide what type of tire is being used. You will need to set this in case you wish to use the TMeasy
/// tire without relaxation (TMeasyNr where Nr stands for no relaxation). See the HalfImplicit and Sundials solver
/// documentation for more details
enum class TireType { TMeasy, TMeasyNr };
namespace d24GPU {

// -----------------------------------------------------------------------------
// Tire Structs
// -----------------------------------------------------------------------------

/// @brief This tire model is an implementation of the TMeasy tire model developed by Prof. Dr. Georg Rill.

/// The TMeasyParam struct holds all the parameters that are required to define a TMeasy tire. See here for more details
/// http://www.tmeasy.de/. It is important to note that this implementation within the d24GPU namespace is exactly
/// the same as the implementation in the d24 namespace. The only difference is that the d24GPU namespace is meant
/// to be used on the GPU where standard library functions are not available.
struct TMeasyParam {
    /// @brief This constructor sets default values for all the parameters in the TMeasyParam struct. The values are an
    /// okay proxy for a truck tire. It is highly recommended that you set the parameters yourself using the
    /// SetTireParamsJSON function that is available within this name space.
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

    /// @brief Copy constructor that takes in a pointer to another TMeasyParam struct and copies all the values into the
    /// new struct
    /// @param other TMeasyParam struct that is copied into the new struct
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
/// http://www.tmeasy.de/. It is important to note that this implementation within the d24GPU namespace is exactly the
/// same as the implementation in the d24 namespace. The only difference is that the d24GPU namespace is meant to be
/// used on the GPU where standard library functions are not available.
struct TMeasyState {
    /// @brief Default constructor that sets all the states to zero
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

    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
    __device__ __host__ TMeasyState(double xe,
                                    double ye,
                                    double xedot,
                                    double yedot,
                                    double omega,
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
        : _xe(xe),
          _ye(ye),
          _xedot(xedot),
          _yedot(yedot),
          _omega(omega),
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

    /// @brief Copy constructor that takes in a pointer to another TMeasyState struct and copies all the values into the
    /// new struct
    /// @param other TMeasyState struct that is copied into the new struct
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

    // States that are to be integrated
    double _xe;      //!< Longitudinal tire deflection
    double _ye;      //!< Lateral tire deflection
    double _xedot;   //!< Longitudinal tire deflection velocity
    double _yedot;   //!< Lateral tire deflection velocity
    double _omega;   //!< Angular velocity of wheel
    double _dOmega;  //!< Angular acceleration of wheel
    double _xt;      //!< Vertical tire compression
    double _dxt;     //!< Vertical tire compression velocity

    // States that we keep track off
    double _rStat;  //!< Loaded tire radius
    // long, lateral and vertical force in tire contact patch frame/ tire frame (after applying steer angle)
    double _fx;   //!< Longitudinal force in tire contact patch frame
    double _fy;   //!< Lateral force in tire contact patch frame
    double _fz;   //!< Vertical force in tire contact patch frame
    double _fxg;  //!< Longitudinal force in tire frame
    double _fyg;  //!< Lateral force in tire frame
    double _fzg;  //!< Vertical force in tire frame

    // velocites in the tire frame (transformed to tire contact path using steer angle)
    // These are obtained from the bond diagram (defines how the vehicle velocities are transformed to tire frame)
    double _vgx;  //!< Longitudinal velocity in tire frame
    double _vgy;  //!< Lateral velocity in tire frame
    double _vgz;  //!< Vertical velocity in tire frame
    // velocites in the tire contact patch frame - used in slip calculations
    double _vsx;     //!< Longitudinal velocity in tire contact patch frame
    double _vsy;     //!< Lateral velocity in tire contact patch frame
    double _vsz;     //!< Vertical velocity in tire contact patch frame
    double _My;      //!< Rolling resistance moment (negative)
    double _engTor;  //!< Torque from engine that we keep track of
};

/// @brief This tire model is an implementation of the TMeasy tire model developed by Prof. Dr. Georg Rill but without
/// relaxation.

/// The TMeasyNrParam (Nr stands for No relaxaation) struct holds all the parameters that are required to
/// define a TMeasyNr tire. The implementation is largely inspired by a similar model in Project Chrono (see code at
/// https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h). It is
/// important to note that this implementation within the d24GPU namespace is exactly the same as the implementation in
/// the d24 namespace. The only difference is that the d24GPU namespace is meant to be used on the GPU where standard
/// library functions are not available.
struct TMeasyNrParam {
    /// @brief This constructor sets default values for all the parameters in the TMeasyNrParam struct. The values are
    /// an okay proxy for a truck tire. It is highly recommended that you set the parameters yourself using the
    /// SetTireParamsJSON function that is available within this name space.
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
    /// @brief Copy constructor that takes in a pointer to another TMeasyNrParam struct and copies all the values into
    /// the new struct
    /// @param other TMeasyNrParam struct that is copied into the new struct
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
/// https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h). It is
/// important to note that this implementation within the d24GPU namespace is exactly the same as the implementation in
/// the d24 namespace. The only difference is that the d24GPU namespace is meant to be used on the GPU where standard
/// library functions are not available.
struct TMeasyNrState {
    /// @brief Default constructor that sets all the states to zero
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
    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
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

    /// @brief Copy constructor that takes in a pointer to another TMeasyNrState struct and copies all the values into
    /// the new struct
    /// @param other TMeasyNrState struct that is copied into the new struct
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
    double _omega;   //!< Angular velocity of wheel
    double _dOmega;  //!< Angular acceleration of wheel
    double _xt;      //!< Vertical tire compression
    double _dxt;     //!< Vertical tire compression velocity

    // States that we keep track off
    double _rStat;  //!< Loaded tire radius
    // long, lateral and vertical force in tire contact patch frame/ tire frame (after applying steer angle)
    double _fx;   //!< Longitudinal force in tire contact patch frame
    double _fy;   //!< Lateral force in tire contact patch frame
    double _fz;   //!< Vertical force in tire contact patch frame
    double _fxg;  //!< Longitudinal force in tire frame
    double _fyg;  //!< Lateral force in tire frame
    double _fzg;  //!< Vertical force in tire frame
    // velocites in the tire frame (transformed to tire contact path using steer angle)
    // These are obtained from the bond diagram (defines how the vehicle velocities are transformed to tire frame)
    double _vgx;  //!< Longitudinal velocity in tire frame
    double _vgy;  //!< Lateral velocity in tire frame
    double _vgz;  //!< Vertical velocity in tire frame
    // velocites in the tire contact patch frame - used in slip calculations
    double _vsx;     //!< Longitudinal velocity in tire contact patch frame
    double _vsy;     //!< Lateral velocity in tire contact patch frame
    double _vsz;     //!< Vertical velocity in tire contact patch frame
    double _My;      //!< Rolling resistance moment (negative)
    double _engTor;  //!< Torque from engine that we keep track of
};

// -----------------------------------------------------------------------------
// Vehicle Structs
// -----------------------------------------------------------------------------
/// @brief Defined here are chassis, engine/motor, powertrain, driveline and steering parameters required for the
/// simulation of a 24 DOF model.

/// The 24 DOF model, which considers the suspension at each corner, offers the same
/// benefits as the 18 DOF model, but it can also predict vehicle heave and pitch motions.
/// Additionally, it provides flexibility in modeling linear springs and dampers. See chapter 2 here for
/// more details https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc. It is important to note that this
/// implementation within the d24GPU namespace is exactly the same as the implementation in the d24 namespace. The only
/// difference is that the d24GPU namespace is meant to be used on the GPU where standard library functions are not
/// available.
struct VehicleParam {
    /// @brief This constructor sets default values for all the parameters in the VehicleParam struct. The values are an
    /// okay proxy for a truck (namely the HMMWV model).  It is highly recommended that you set the parameters yourself
    /// using the setVehParamsJSON function that is available within this name space.
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
    /// @brief Copy constructor that takes in a pointer to another VehicleParam struct and copies all the values into
    /// the new struct
    /// @param other VehicleParam struct that is copied into the new struct
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

    // Declaration of all the 24 DOF vehicle parameters pretty much the same as the 18 DOF model
    double _a;     //!< Distance of C.G to front axle (m)
    double _b;     //!< Distance of C.G to rear axle (m)
    double _h;     //!< Height of C.G
    double _m;     //!< Total vehicle mass (kg)
    double _jz;    //!< Yaw moment of inertia (kg.m^2)
    double _jx;    //!< Roll inertia (kg.m^2)
    double _jy;    //!< Pitch inertia (kg.m^2)
    double _cf;    //!< Front track width (m)
    double _cr;    //!< Rear track width (m)
    double _muf;   //!< Front unsprung mass (kg)
    double _mur;   //!< Rear unsprung mass (kg)
    double _hrcf;  //!< Front roll centre height below C.G (m)
    double _hrcr;  //!< Rear roll centre height below C.G (m)

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
     *  _steerMap points to a array of MapEntry's where the x component of the MapEntry is the normalized steering input
     * (between -1 and 1 where negative implies a left turn) and the y component is the wheel angle. For steerng inputs
     * between the values specified by the MapEntry's, the wheel angle is linearly interpolated. Set using the
     * "steerMap" key in the JSON file.
     */
    MapEntry* _steerMap;
    int _steerMapSize;  //!< Size of the steering map - this variable is unique to the GPU implementation as we need to
                        //!< know the size of the map to allocate memory for it.

    double _maxSteer;  //!< In case _nonLinearSteer is set to 0, this is the maximum wheel angle that can be achieved
                       //!< (in radians). Values at other steering wheel inputs are then linearly interpolated.

    double _crankInertia;  //!< Inertia of the engine/motor crankshaft
    // some gear parameters
    double* _gearRatios;  //!< Pointer to an array of gear ratios
    int _noGears;         //!< No. of gears in the vehicle -> Set automatically when the gear ratios are set

    /**
     * @brief A shift map defines the RPMs at which the various gears shift.
     *
     * _shiftMap points to a array of MapEntry's where the x component of the MapEntry is the down shift RPM and the y
     * component is the upshift RPM. The user can either specify the shift map for all the gears or can specify an
     * "upshiftRPM" and a "downshiftRPM". These two values are then applied for all the gears (see json files for
     * examples)
     */
    MapEntry* _shiftMap;

    bool _tcbool;  //!< Boolean that checks for the presence of a torque converter. Can be set using "tcBool" in the
                   //!< JSON file. Defaults to 0 if not specified.

    double _maxBrakeTorque;  //!< The maximum braking torque (Nm) that the brakes apply to the wheels. Can be set using
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
     * _powertrainMap points to a array of MapEntry's where the x component of the MapEntry is the engine crankshaft RPM
     and the y
     * component is the torque. For RPMs between the values specified by the MapEntry's, the torque is linearly
     interpolated. Set using the "torqueMap" key in the JSON file.
     */
    MapEntry* _powertrainMap;
    int _powertrainMapSize;  //!< Size of the powertrain map - set automatically when the powertrain map is set
    /**
     * @brief _lossesMap stores the powertrain losses map and can be set using the JSON file using the "lossesMap" key.
     * _lossesMap points to a array of MapEntry's where the x component of the MapEntry is the engine crankshaft RPM and
     * the y component is the losses in Nm. For RPMs between the values specified by the MapEntry's, the losses are
     * linearly interpolated. The losses are subtracted from the torque obtained from the powertrain map after throttle
     * is applied. See function driveTorque for more details.
     * Set using the "lossesMap" key in the JSON file.
     */
    MapEntry* _lossesMap;
    int _lossesMapSize;  //!< Size of the losses map - set automatically when the losses map is set
    /**
     * @brief In case a torque converter is part of the model, the capacity map needs to be defined.
     *
     * _CFmap points to a array of MapEntry's where the x component of the MapEntry is the speed ratio between the
     * driven and driving shafts (whether the engine crank is driven or driving depends on if there is a throttle
     * applied) and the y component is the capacity factor. For speed ratios between the values specified by the
     * MapEntry's, the capacity factor is linearly interpolated.
     * Set using the "capacityFactorMap" key in the JSON file.
     */
    MapEntry* _CFmap;
    int _CFmapSize;  //!< Size of the capacity factor map - set automatically when the capacity factor map is set

    /**
     * @brief In case a torque converter is part of the model, the torque ratio map needs to be defined.
     *
     * _TRmap points to a array of MapEntry's where the x component of the MapEntry is the speed ratio between the
     * driven and driving shafts (whether the engine crank is driven or driving depends on if there is a throttle
     * applied) and the y component is the torque ratio. For speed ratios between the values specified by the
     * MapEntry's, the torque ratio is linearly interpolated. Set using the "torqueRatioMap" key in the JSON file.
     */
    MapEntry* _TRmap;
    int _TRmapSize;  //!< Size of the torque ratio map - set automatically when the torque ratio map is set
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
/// for the simulation.

/// Apart from the states that are updated by time integration, this struct also holds "non-states"
///  such as accelerations, forces and torques that are not updated by time integration. See here for more details
///  https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc. important to note that this implementation within the
///  d24GPU namespace is exactly the same as the implementation in the d24 namespace. The only difference is that the
///  d24GPU namespace is meant to be used on the GPU where standard library functions are not available.
struct VehicleState {
    /// @brief Default constructor that sets all the states to zero
    __device__ __host__ VehicleState()
        : _x(0.),
          _y(0.),
          _z(0.),
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
          _crankOmega(0.),
          _dOmega_crank(0.),
          _current_gr(0) {}

    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
    __device__ __host__ VehicleState(double x,
                                     double y,
                                     double z,
                                     double u,
                                     double v,
                                     double w,
                                     double psi,
                                     double wz,
                                     double phi,
                                     double wx,
                                     double theta,
                                     double wy,
                                     double udot,
                                     double vdot,
                                     double wdot,
                                     double wxdot,
                                     double wydot,
                                     double wzdot,
                                     double dtheta,
                                     double dphi,
                                     double dpsi,
                                     double crankOmega,
                                     double dOmega_crank,
                                     int current_gr)
        : _x(x),
          _y(y),
          _z(z),
          _u(u),
          _v(v),
          _w(w),
          _psi(psi),
          _wz(wz),
          _phi(phi),
          _wx(wx),
          _theta(theta),
          _wy(wy),
          _udot(udot),
          _vdot(vdot),
          _wdot(wdot),
          _wxdot(wxdot),
          _wydot(wydot),
          _wzdot(wzdot),
          _dtheta(dtheta),
          _dphi(dphi),
          _dpsi(dpsi),
          _crankOmega(crankOmega),
          _dOmega_crank(dOmega_crank),
          _current_gr(current_gr) {}
    /// @brief Copy constructor that takes in a pointer to another VehicleState struct and copies all the values into
    /// the new struct
    /// @param other VehicleState struct that is copied into the new struct
    __device__ __host__ VehicleState(const VehicleState* other)
        : _x(other->_x),
          _y(other->_y),
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
          _crankOmega(other->_crankOmega),
          _dOmega_crank(other->_dOmega_crank),
          _current_gr(other->_current_gr) {}
    // States
    double _x;    //!< Global x position
    double _y;    //!< Global y position
    double _z;    //!< Global z posiiton
    double _u;    //!< Chassis x velocity in G-RF
    double _v;    //!< Chassis y velocity in G-RF
    double _w;    //!< Chassis z velocity in G-RF
    double _psi;  //!< Yaw angle of chassis (rotation about z axis of G-RF) - this is the angle that is used to rotate
                  //!< the chassis frame to the global frame
    double _wz;   //!< Yaw rate of chassis
    double _phi;  //!< Roll angle of chassis (rotation about x axis of G-RF) - this is the angle that is used to rotate
                  //!< the chassis frame to the global frame
    double _wx;   //!< Roll rate of chassis
    double _theta;  //!< Pitch angle of chassis (rotation about y axis of G-RF) - this is the angle that is used to
                    //!< rotate the chassis frame to the global frame
    double _wy;     //!< Pitch rate of chassis

    // acceleration states
    double _udot;    //!< Chassis x acceleration in G-RF
    double _vdot;    //!< Chassis y acceleration in G-RF
    double _wdot;    //!< Chassis z acceleration in G-RF
    double _wxdot;   //!< Roll acceleration of chassis
    double _wydot;   //!< Pitch acceleration of chassis
    double _wzdot;   //!< Yaw acceleration of chassis
    double _dtheta;  //!< Change in pitch angle of chassis after frame transformation
    double _dphi;    //!< Change in roll angle of chassis after frame transformation
    double _dpsi;    //!< Change in yaw angle of chassis after frame transformation

    // Powertrain states
    double _crankOmega;    //!< Crankshaft angular velocity
    double _dOmega_crank;  //!< Crankshaft angular acceleration
    int _current_gr;       //!< Current gear
};

// -----------------------------------------------------------------------------
// Suspension Structs
// -----------------------------------------------------------------------------

/// @brief The SuspensionParam struct holds the spring stiffness and damper coefficients for the suspension.
struct SuspensionParam {
    /// @brief Default constructor that sets the parameters to the default values of a stiff suspension system
    __device__ __host__ SuspensionParam() : _ks(150062), _bs(19068) {}
    /// @brief Copy constructor that takes in a pointer to another SuspensionParam struct and copies all the values into
    /// the new struct
    /// @param other SuspensionParam struct that is copied into the new struct
    __device__ __host__ SuspensionParam(const SuspensionParam* other) : _ks(other->_ks), _bs(other->_bs) {}
    // Parameters - Maybe later we can add map functionality
    double _ks;  // suspension stiffness
    double _bs;  // suspension damping coefficient
};

/// @brief The SuspensionState are all the states and the corresponding accelerations, forces and moments that are part
/// of a broad "suspension" model.

/// This includes variables such as the unsprung mass velocities and accelerations, the
/// instantaneous spring deflections and velocities, the link load transfer forces and the moments transferred to the
/// sprung mass. For more details about the implementation of the suspension model, see chapter 2 here
/// https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc. It is important to note that this implementation
/// within the d24GPU namespace is exactly the same as the implementation in the d24 namespace. The only difference is
/// that the d24GPU namespace is meant to be used on the GPU where standard library functions are not available.
struct SuspensionState {
    /// @brief Default constructor that sets all the states to zero
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
    /// @brief Constructor that takes in all the states and sets them to the values that are passed in. Note, its
    /// usually very difficult to set all the states to physical meaningful values while ensuring that the tire is in
    /// equilibrium. This is also largely untested and its thus recommended to initiaize the states to 0 or use the copy
    /// constructor below
    __device__ __host__ SuspensionState(double xs,
                                        double xsi,
                                        double dxs,
                                        double ls,
                                        double lsi,
                                        double us,
                                        double vs,
                                        double ws,
                                        double dus,
                                        double dvs,
                                        double dws,
                                        double uu,
                                        double vu,
                                        double wu,
                                        double duu,
                                        double dvu,
                                        double dwu,
                                        double fxs,
                                        double fys,
                                        double fzs,
                                        double fzd,
                                        double mx,
                                        double my,
                                        double mz)
        : _xs(xs),
          _xsi(xsi),
          _dxs(dxs),
          _ls(ls),
          _lsi(lsi),
          _us(us),
          _vs(vs),
          _ws(ws),
          _dus(dus),
          _dvs(dvs),
          _dws(dws),
          _uu(uu),
          _vu(vu),
          _wu(wu),
          _duu(duu),
          _dvu(dvu),
          _dwu(dwu),
          _fxs(fxs),
          _fys(fys),
          _fzs(fzs),
          _fzd(fzd),
          _mx(mx),
          _my(my),
          _mz(mz) {}
    /// @brief Copy constructor that takes in a pointer to another SuspensionState struct and copies all the values into
    /// the new struct
    /// @param other SuspensionState struct that is copied into the new struct
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
    double _xs;   //!< Instantaneous spring compression
    double _xsi;  //!< Initial spring compression
    double _dxs;  //!< Instantaneous sping compression accelerations
    double _ls;   //!< Instantaneous struct length
    double _lsi;  //!< Initial struct length

    double _us;   //!< Struct velocity in the x direction (G-RF)
    double _vs;   //!< Struct velocity in the y direction (G-RF)
    double _ws;   //!< Struct velocity in the z direction (G-RF)
    double _dus;  //!< Struct acceleration in the x direction (G-RF)
    double _dvs;  //!< Struct acceleration in the y direction (G-RF)
    double _dws;  //!< Struct acceleration in the z direction (G-RF)
    double _uu;   //!< Unsprung velocity in the x direction (G-RF)
    double _vu;   //!< Unsprung velocity in the y direction (G-RF)
    double _wu;   //!< Unsprung velocity in the z direction (G-RF)
    double _duu;  //!< Unsprung acceleration in the x direction (G-RF)
    double _dvu;  //!< Unsprung acceleration in the y direction (G-RF)
    double _dwu;  //!< Unsprung acceleration in the z direction (G-RF)

    // Forces and moments
    double _fxs;  //!< Forces transferred to sprung mass in the x direction (G-RF)
    double _fys;  //!< Forces transferred to sprung mass in the y direction (G-RF)
    double _fzs;  //!< Forces transferred to sprung mass in the z direction (G-RF)
    double _fzd;  //!< Link load transfer forces in the z directions (jacking forces)
    double _mx;   //!< Moment transferred to sprung mass in the x direction (G-RF)
    double _my;   //!< Moment transferred to sprung mass in the y direction (G-RF)
    double _mz;   //!< Moment transferred to sprung mass in the z direction (G-RF)
};

// -----------------------------------------------------------------------------
// Data structures that help handing multiple vehicles as needed in GPU version
// -----------------------------------------------------------------------------

/// @brief The SimData struct holds the parameters of the vehicle, tire and suspension needed in kernel along with the
/// driver inputs (steering, throttle, brake) in case specified at the beginning of the simulation.

/// In essence, this
/// stores all the "data" required to simulate 1 vehicle on the GPU. This is something largely the user does not have to
/// worry about.
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
    VehicleParam _veh_params;       //!< Vehicle parameters
    TMeasyParam _tireTM_params;     //!< Tire parameters (TMeasy)
    SuspensionParam _sus_params;    //!< Suspension parameters
    DriverInput* _driver_data;      //!< Driver inputs
    unsigned int _driver_data_len;  //!< Length of driver inputs
};
/// @brief  The SimDataNr is identical to SimData except that it uses the TMeasyNr tire model instead of the TMeasy tire
/// model.
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
    VehicleParam _veh_params;        //!< Vehicle parameters
    TMeasyNrParam _tireTMNr_params;  //!< Tire parameters (TMeasyNr)
    SuspensionParam _sus_params;     //!< Suspension parameters
    DriverInput* _driver_data;       //!< Driver inputs
    unsigned int _driver_data_len;   //!< Length of driver inputs
};

/// @brief The SimState struct holds the states of the vehicle, tire and suspension needed in kernel.

/// In essence, this
/// stores all the "states" of 1 vehicle simulated on the GPU. The user can use this to get the states of the vehicle at
/// any point in time during the simulation through the solver (see d24SolverHalfImplicitGPU class)
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

    VehicleState _v_states;     //!< Vehicle states
    TMeasyState _tirelf_st;     //!< Tire states (TMeasy Left Front (LF))
    TMeasyState _tirerf_st;     //!< Tire states (TMeasy Right Front (RF))
    TMeasyState _tirelr_st;     //!< Tire states (TMeasy Left Rear (LR))
    TMeasyState _tirerr_st;     //!< Tire states (TMeasy Right Rear (RR))
    SuspensionState _suslf_st;  //!< Suspension states (Left Front (LF))
    SuspensionState _susrf_st;  //!< Suspension states (Right Front (RF))
    SuspensionState _suslr_st;  //!< Suspension states (Left Rear (LR))
    SuspensionState _susrr_st;  //!< Suspension states (Right Rear (RR))
};
/// @brief The SimStateNr struct is identical to SimState except that it uses the TMeasyNr tire model instead of the
/// TMeasy tire model.
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

    VehicleState _v_states;     //!< Vehicle states
    TMeasyNrState _tirelf_st;   //!< Tire states (TMeasyNr Left Front (LF))
    TMeasyNrState _tirerf_st;   //!< Tire states (TMeasyNr Right Front (RF))
    TMeasyNrState _tirelr_st;   //!< Tire states (TMeasyNr Left Rear (LR))
    TMeasyNrState _tirerr_st;   //!< Tire states (TMeasyNr Right Rear (RR))
    SuspensionState _suslf_st;  //!< Suspension states (Left Front (LF))
    SuspensionState _susrf_st;  //!< Suspension states (Right Front (RF))
    SuspensionState _suslr_st;  //!< Suspension states (Left Rear (LR))
    SuspensionState _susrr_st;  //!< Suspension states (Right Rear (RR))
};

// -----------------------------------------------------------------------------
// Initalize functions
// -----------------------------------------------------------------------------
/// @brief Initializes the nominal and critical value for the vertical force based on tire parameters
/// @param t_params Pointer to the TMeasytire parameters
__host__ void tireInit(TMeasyParam* t_params);
/// @brief Overload of tireInit for TMeasyNr tire model
/// @param t_params Pointer to the TMeasyNr tire parameters
__host__ void tireInit(TMeasyNrParam* t_params);

/// @brief Based on the static loads, this function initializes the tire compression, spring compression and the length
/// of the struct
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasy tire parameters
/// @param sus_params Suspension parameters
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
/// @brief Overload of initializeTireSus for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasy tire parameters
/// @param sus_params Suspension parameters
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
/// @brief Transform all vehicle velocites and acclerations to struct and unsprung mass velocites
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
__device__ void vehToSusTransform(const VehicleState* v_states,
                                  const TMeasyState* tirelf_st,
                                  const TMeasyState* tirerf_st,
                                  const TMeasyState* tirelr_st,
                                  const TMeasyState* tirerr_st,
                                  SuspensionState* suslf_st,
                                  SuspensionState* susrf_st,
                                  SuspensionState* suslr_st,
                                  SuspensionState* susrr_st,
                                  const VehicleParam* v_params);
/// @brief Overload of vehToSusTransform for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
__device__ void vehToSusTransform(const VehicleState* v_states,
                                  const TMeasyNrState* tirelf_st,
                                  const TMeasyNrState* tirerf_st,
                                  const TMeasyNrState* tirelr_st,
                                  const TMeasyNrState* tirerr_st,
                                  SuspensionState* suslf_st,
                                  SuspensionState* susrf_st,
                                  SuspensionState* suslr_st,
                                  SuspensionState* susrr_st,
                                  const VehicleParam* v_params);
/// @brief Transform all struct and unsprung mass velocites tire velocities
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
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
/// @brief Overload of vehToTireTransform for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
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
/// @brief Transform forces generated in the contact patch to forces in inertial frame (G-RF)
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
__device__ void tireToVehTransform(const VehicleState* v_states,
                                   TMeasyState* tirelf_st,
                                   TMeasyState* tirerf_st,
                                   TMeasyState* tirelr_st,
                                   TMeasyState* tirerr_st,
                                   const VehicleParam* v_params,
                                   double steering);
/// @brief Overload of tireToVehTransform for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param steering Steering input to get the angle of the wheel which gives the orientation of the wheel frame
__device__ void tireToVehTransform(const VehicleState* v_states,
                                   TMeasyNrState* tirelf_st,
                                   TMeasyNrState* tirerf_st,
                                   TMeasyNrState* tirelr_st,
                                   TMeasyNrState* tirerr_st,
                                   const VehicleParam* v_params,
                                   double steering);
/// @brief The forces from the tire to the struct in the inertial frame (G-RF) this then acts on the Chassis body and
/// moves the vehicle
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasy tire parameters
/// @param sus_params Suspension parameters
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
/// @brief Overload of computeForcesThroughSus for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasyNr tire parameters
/// @param sus_params Suspension parameters
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
/// @brief Helper function to provide the tire force (f) and tire force / combined slip (fos) for the combined slip
/// model of the TMeasy and TMeasyNr tire models.

/// See page 78 of book Road Vehicle Dynamics: Fundamentals and Modeling by Georg Rill for more details about other
/// input parameters
__device__ void tmxy_combined(double* f, double* fos, double s, double df0, double sm, double fm, double ss, double fs);

/// @brief Computes the combined coulomb force for the TMeasyNr tire model

/// This force provides the stability at low speeds and is belnded with the slip force provided by the tmxy_combined
/// function.
__device__ void
computeCombinedColumbForce(double* fx, double* fy, double mu, double vsx, double vsy, double fz, double vcoulomb);

/// @brief Computes the tire forces for the TMeasy tire model in the tire contact patch frame (T-RF)

/// The tire slip force produces a lateral and longitudinal tire deflection which produces the forces. A lot more detail
/// of the formulation can be found in the book Road Vehicle Dynamics: Fundamentals and Modeling by Georg Rill
/// @param v_states Vehicle States
/// @param t_states TMeasy tire states
/// @param v_params Vehicle Parameters
/// @param t_params TMeasy tire parameters
/// @param steering Steering input used to calculate the lateral tire slip
__device__ void computeTireRHS(const VehicleState* v_states,
                               TMeasyState* t_states,
                               const VehicleParam* v_params,
                               const TMeasyParam* t_params,
                               double steering);
/// @brief Computes the tire forces for the TMeasyNr tire model in the tire contact patch frame (T-RF)

/// For the TMeasyNr tire model, since there is no relaxation, the tire forces from slip are blended with the tire
/// forces from coulomb friction. The blend coefficient depends on the longitudinal slip velocity of the tire. This model
/// is an approximation of the original TMeasy tire model and is inspired by the Project Chrono implementation (see code
/// at https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h).
/// @param v_states Vehicle States
/// @param t_states TMeasyNr tire states
/// @param v_params Vehicle Parameters
/// @param t_params TMeasyNr tire parameter
/// @param steering Steering input used to calculate the lateral tire slip
__device__ void computeTireRHS(const VehicleState* v_states,
                               TMeasyNrState* t_states,
                               const VehicleParam* v_params,
                               const TMeasyNrParam* t_params,
                               double steering);
/// @brief Computes the tire compression RHS that is to be integrated. This function is just an extension of
/// computeTireRHS but also requires the suspension states.

/// @param v_states Vehicle States
/// @param tirelf_st TMesay tire states (Left Front (LF))
/// @param tirerf_st TMesay tire states (Left Front (RF))
/// @param tirelr_st TMesay tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
__device__ void computeTireCompressionVelocity(const VehicleState* v_states,
                                               TMeasyState* tirelf_st,
                                               TMeasyState* tirerf_st,
                                               TMeasyState* tirelr_st,
                                               TMeasyState* tirerr_st,
                                               const SuspensionState* suslf_st,
                                               const SuspensionState* susrf_st,
                                               const SuspensionState* suslr_st,
                                               const SuspensionState* susrr_st);

/// @brief Overload of computeTireCompressionVelocity for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMesayNr tire states (Left Front (LF))
/// @param tirerf_st TMesayNr tire states (Left Front (RF))
/// @param tirelr_st TMesayNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
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
/// @brief Computes the suspension deflection RHS that are to be integrated
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param sus_params Suspension Parameters
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
/// @brief Overload of computeSusRHS for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param sus_params Suspension Parameters
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
/// @brief Computes the Engine drive torque based on throttle input, engine omega _throttleMod, the _powertrainMap and
/// the _lossesMap.
/// @param v_params Vehicle Parameters
/// @param throttle Throttle input [0-1]
/// @param omega Engine angular velocity (rad/s)
__device__ double driveTorque(const VehicleParam* v_params, const double throttle, const double omega);

/// @brief Computes the Engine braking torque based on brake input and _maxBrakeTorque
/// @param v_params Vehicle Parameters
/// @param brake Brake input [0-1]
__device__ inline double brakeTorque(const VehicleParam* v_params, const double brake) {
    return v_params->_maxBrakeTorque * brake;
}
/// @brief  Helper function that calculates the torque split to each tire based on the differential max bias and the
/// tires relative angular velocity
/// @param torque Torque to differential
/// @param max_bias Bias that determines the impact of relative angular velocity on torque split
/// @param speed_left Angular velocity of left tire
/// @param speed_right Angular velocity of right tire
/// @param torque_left Torque provided to the left tire
/// @param torque_right Torque provided to the right tire
__device__ void differentialSplit(double torque,
                                  double max_bias,
                                  double speed_left,
                                  double speed_right,
                                  double* torque_left,
                                  double* torque_right);
/// @brief Computes the Crank-Shaft angular acceleration (if there is a torque converter) as well as the angular wheel
/// accelerations that are integrated to provide the wheel angular velocities.

/// In case there is no torque converter, the crank-shaft angular velocity is calculated solely based on the wheel
/// angular velocities (not integrated). This function is also where the velocities go from the wheel to the engine
/// (through the differential, gear box, torque converter (optional)) and the torques go from the engine to the wheel
/// (through the differential, gear box, torque converter (optional)).
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMesay tire states (Left Front (LR))
/// @param tirerr_st TMesay tire states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMesay tire parameters
/// @param controls Vehicle controls (throttle, brake, steering)
__device__ void computePowertrainRHS(VehicleState* v_states,
                                     TMeasyState* tirelf_st,
                                     TMeasyState* tirerf_st,
                                     TMeasyState* tirelr_st,
                                     TMeasyState* tirerr_st,
                                     const VehicleParam* v_params,
                                     const TMeasyParam* t_params,
                                     const DriverInput* controls);
/// @brief computePowertrainRHS overload for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMesayNr tire states (Left Front (LR))
/// @param tirerr_st TMesayNr tire states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasyNr tire parameters
/// @param controls Vehicle controls (throttle, brake, steering)
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
/// @brief Computes the vehicle linear and angular accelerations that are to be integrated
/// @param v_states Vehicle States
/// @param tirelf_st TMeasy tire states (Left Front (LF))
/// @param tirerf_st TMeasy tire states (Left Front (RF))
/// @param tirelr_st TMeasy tire states (Left Front (LR))
/// @param tirerr_st TMeasy tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasy tire parameters
/// @param sus_params Suspension parameters
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
/// @brief computeVehicleRHS overload for TMeasyNr tire model
/// @param v_states Vehicle States
/// @param tirelf_st TMeasyNr tire states (Left Front (LF))
/// @param tirerf_st TMeasyNr tire states (Left Front (RF))
/// @param tirelr_st TMeasyNr tire states (Left Front (LR))
/// @param tirerr_st TMeasyNr tire states (Left Front (RR))
/// @param suslf_st Suspension states (Left Front (LF))
/// @param susrf_st Suspension states (Left Front (RF))
/// @param suslr_st Suspension states (Left Front (LR))
/// @param susrr_st Suspension states (Left Front (RR))
/// @param v_params Vehicle Parameters
/// @param t_params TMeasyNr tire parameters
/// @param sus_params Suspension parameters
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
/// @brief Parses the tire parameters from a JSON file for the TMeasy tire model. Please see implementation and/or
/// example json files for appropriate keys and format.
/// @param t_params TMeasy tire parameters
/// @param fileName Path to JSON file (see demo for example)
__host__ void setTireParamsJSON(TMeasyParam& t_params, const char* fileName);

/// @brief Parses the tire parameters from a JSON file for the TMeasyNr tire model. Please see implementation and/or
/// example json files for appropriate keys and format.

/// The TMeasyNr follows a different approach to setting tire parameters as it only requires high level tire parameters
/// to be set by the user. However, the user can still set the tire parameters directly if they wish to do so by setting
/// the "highLevelParams" key to false.
/// @param t_params TMeasyNr tire parameters
/// @param fileName Path to JSON file (see demo for example)
__host__ void setTireParamsJSON(TMeasyNrParam& t_params, const char* fileName);

/// @brief Helper function to compute the max tire load from the load index
/// @param li Load index
__host__ double GetTireMaxLoad(unsigned int li);
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
__host__ void GuessTruck80Par(unsigned int li,
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
__host__ void GuessTruck80Par(double tireLoad,
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
__host__ void GuessPassCar70Par(unsigned int li,
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
/// @brief Parses the suspension parameters from a JSON file for the TMeasy tire model.
/// @param sus_params Suspension parameters
/// @param fileName Path to JSON file (see demo for example)
__host__ void setSuspensionParamsJSON(SuspensionParam& sus_params, const char* fileName);
// --------
// Vehicle
// --------
/// @brief Parses the vehicle parameters from a JSON file for the TMeasy tire model.
/// @param v_params Vehicle parameters
/// @param fileName Path to JSON file (see demo for example)
__host__ void setVehParamsJSON(VehicleParam& v_params, const char* fileName);
}  // namespace d24GPU

#endif
