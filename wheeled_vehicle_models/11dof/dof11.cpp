#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof11.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace d11;

// Returns drive torque at a given omega
double d11::driveTorque(const VehicleParam& v_params, const double throttle, const double motor_speed) {
    // need to do this because lower_bound does not take a constant vector
    // std::vector<MapEntry> powertrain_map = v_params._powertrainMap;
    // std::vector<MapEntry> losses_map = v_params._lossesMap;

    double motor_torque = 0.;
    // If we have throttle modulation like in a motor
    if (v_params._throttleMod) {
        std::vector<MapEntry> powertrain_map = v_params._powertrainMap;

        size_t len = v_params._powertrainMap.size();
        for (size_t i = 0; i < len; i++) {
            powertrain_map[i]._x = v_params._powertrainMap[i]._x * throttle;
            powertrain_map[i]._y = v_params._powertrainMap[i]._y * throttle;
        }

        // interpolate in the torque map to get the torque at this paticular
        // speed
        motor_torque = getMapY(powertrain_map, motor_speed);
        double motor_losses = getMapY(v_params._lossesMap, motor_speed);
        motor_torque = motor_torque + motor_losses;
    } else {  // Else we don't multiply the map but just the output torque
        motor_torque = getMapY(v_params._powertrainMap, motor_speed);
        double motor_losses = getMapY(v_params._lossesMap, motor_speed);
        motor_torque = motor_torque * throttle + motor_losses;
    }
    return motor_torque;
}

// Loads being utilized correctly here?
void d11::vehToTireTransform(TMeasyState& tiref_st,
                             TMeasyState& tirer_st,
                             const VehicleState& v_states,
                             const std::vector<double>& loads,
                             const VehicleParam& v_params,
                             double steering) {
    // Get the steering considering the mapping might be non linear
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    // left front
    tiref_st._fz = loads[0];
    tiref_st._vsy = v_states._v + v_states._wz * v_params._a;
    tiref_st._vsx = v_states._u * std::cos(delta) + tiref_st._vsy * std::sin(delta);

    // right front
    tirer_st._fz = loads[1];
    tirer_st._vsy = v_states._v - v_states._wz * v_params._b;
    tirer_st._vsx = v_states._u;
}

// Loads being utilized correctly here?
void d11::vehToTireTransform(TMeasyNrState& tiref_st,
                             TMeasyNrState& tirer_st,
                             const VehicleState& v_states,
                             const std::vector<double>& loads,
                             const VehicleParam& v_params,
                             double steering) {
    // Get the steering considering the mapping might be non linear
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    // left front
    tiref_st._fz = loads[0];
    tiref_st._vsy = v_states._v + v_states._wz * v_params._a;
    tiref_st._vsx = v_states._u * std::cos(delta) + tiref_st._vsy * std::sin(delta);

    // right front
    tirer_st._fz = loads[1];
    tirer_st._vsy = v_states._v - v_states._wz * v_params._b;
    tirer_st._vsx = v_states._u;
}

void d11::tireToVehTransform(TMeasyState& tiref_st,
                             TMeasyState& tirer_st,
                             const VehicleState& v_states,
                             const VehicleParam& v_params,
                             double steering) {
    // Get the steering considering the mapping might be non linear
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    double _fx, _fy;

    // Front tires steer by delta
    _fx = tiref_st._fx * std::cos(delta) - tiref_st._fy * std::sin(delta);
    _fy = tiref_st._fx * std::sin(delta) + tiref_st._fy * std::cos(delta);

    tiref_st._fx = _fx;
    tiref_st._fy = _fy;

    // rear tires - No steer so no need to transform
}
void d11::tireToVehTransform(TMeasyNrState& tiref_st,
                             TMeasyNrState& tirer_st,
                             const VehicleState& v_states,
                             const VehicleParam& v_params,
                             double steering) {
    // Get the steering considering the mapping might be non linear
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    double _fx, _fy;

    // Front tires steer by delta
    _fx = tiref_st._fx * std::cos(delta) - tiref_st._fy * std::sin(delta);
    _fy = tiref_st._fx * std::sin(delta) + tiref_st._fy * std::cos(delta);

    tiref_st._fx = _fx;
    tiref_st._fy = _fy;

    // rear tires - No steer so no need to transform
}

/////////////////////////////////////////////////////////////////////////////
/// Tire Functions
////////////////////////////////////////////////////////////

// Code for the TM easy tire model implemented with the 8DOF model
void d11::tireInit(TMeasyParam& t_params) {
    // calculates some critical values that are needed
    t_params._fzRdynco = (t_params._pn * (t_params._rdyncoP2n - 2.0 * t_params._rdyncoPn + 1.)) /
                         (2. * (t_params._rdyncoP2n - t_params._rdyncoPn));

    t_params._rdyncoCrit = InterpL(t_params._fzRdynco, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
}
void d11::tireInit(TMeasyNrParam& t_params) {
    // calculates some critical values that are needed
    t_params._fzRdynco = (t_params._pn * (t_params._rdyncoP2n - 2.0 * t_params._rdyncoPn + 1.)) /
                         (2. * (t_params._rdyncoP2n - t_params._rdyncoPn));

    t_params._rdyncoCrit = InterpL(t_params._fzRdynco, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
}

void d11::tmxy_combined(double& f, double& fos, double s, double df0, double sm, double fm, double ss, double fs) {
    double df0loc = 0.0;
    if (sm > 0.0) {
        df0loc = std::max(2.0 * fm / sm, df0);
    }

    if (s > 0.0 && df0loc > 0.0) {  // normal operating conditions
        if (s > ss) {               // full sliding
            f = fs;
            fos = f / s;
        } else {
            if (s < sm) {  // adhesion
                double p = df0loc * sm / fm - 2.0;
                double sn = s / sm;
                double dn = 1.0 + (sn + p) * sn;
                f = df0loc * sm * sn / dn;
                fos = df0loc / dn;
            } else {
                double a = std::pow(fm / sm, 2.0) / (df0loc * sm);  // parameter from 2. deriv. of f @ s=sm
                double sstar = sm + (fm - fs) / (a * (ss - sm));    // connecting point
                if (sstar <= ss) {                                  // 2 parabolas
                    if (s <= sstar) {
                        // 1. parabola sm < s < sstar
                        f = fm - a * (s - sm) * (s - sm);
                    } else {
                        // 2. parabola sstar < s < ss
                        double b = a * (sstar - sm) / (ss - sstar);
                        f = fs + b * (ss - s) * (ss - s);
                    }
                } else {
                    // cubic fallback function
                    double sn = (s - sm) / (ss - sm);
                    f = fm - (fm - fs) * sn * sn * (3.0 - 2.0 * sn);
                }
                fos = f / s;
            }
        }
    } else {
        f = 0.0;
        fos = 0.0;
    }
}
void d11::computeCombinedColumbForce(double& fx,
                                     double& fy,
                                     double mu,
                                     double vsx,
                                     double vsy,
                                     double fz,
                                     double vcoulomb) {
    fx = tanh(-2.0 * vsx / vcoulomb) * fz * mu;
    fy = tanh(-2.0 * vsy / vcoulomb) * fz * mu;

    // Nromalize F to the circle
    if (std::hypot(fx, fy) > fz * mu) {
        double f = fz * mu / std::hypot(fx, fy);
        fx *= f;
        fy *= f;
    }
}

void d11::computeTireLoads(std::vector<double>& loads,
                           const VehicleState& v_states,
                           const VehicleParam& v_params,
                           const TMeasyParam& t_params) {
    double huf = t_params._r0;
    double hur = t_params._r0;

    // Static vertical load transfer based on d"Almberts principle
    double Z1 = (v_params._m * G * v_params._b) / (2. * (v_params._a + v_params._b)) + (v_params._muf * G) / 2.;
    double Z2 = ((v_params._m * v_params._h + v_params._muf * huf + v_params._mur * hur) *
                 (v_states._udot - v_states._wz * v_states._v)) /
                (2. * (v_params._a + v_params._b));

    loads[0] = (Z1 - Z2) > 0. ? (Z1 - Z2) : 0.;

    double Z3 = (v_params._m * G * v_params._a) / (2. * (v_params._a + v_params._b)) + (v_params._mur * G) / 2.;

    loads[1] = (Z3 + Z2) > 0. ? (Z3 + Z2) : 0.;
}

void d11::computeTireLoads(std::vector<double>& loads,
                           const VehicleState& v_states,
                           const VehicleParam& v_params,
                           const TMeasyNrParam& t_params) {
    double huf = t_params._r0;
    double hur = t_params._r0;

    // Static vertical load transfer based on d"Almberts principle
    double Z1 = (v_params._m * G * v_params._b) / (2. * (v_params._a + v_params._b)) + (v_params._muf * G) / 2.;
    double Z2 = ((v_params._m * v_params._h + v_params._muf * huf + v_params._mur * hur) *
                 (v_states._udot - v_states._wz * v_states._v)) /
                (2. * (v_params._a + v_params._b));

    loads[0] = (Z1 - Z2) > 0. ? (Z1 - Z2) : 0.;

    double Z3 = (v_params._m * G * v_params._a) / (2. * (v_params._a + v_params._b)) + (v_params._mur * G) / 2.;

    loads[1] = (Z3 + Z2) > 0. ? (Z3 + Z2) : 0.;
}

// split bool decides if its 2wd or 1wd
// whichWheels decides if its rear wheel drive (True) or front wheel drive
void d11::differentialSplit(double torque,
                            double max_bias,
                            double speed_rear,
                            double speed_front,
                            double& torque_rear,
                            double& torque_front,
                            bool split,
                            bool whichWheels) {
    double diff = std::abs(speed_rear - speed_front);

    if (split) {
        // The bias grows from 1 at diff=0.25 to max_bias at diff=0.5
        double bias = 1;
        if (diff > 0.5)
            bias = max_bias;
        else if (diff > 0.25)
            bias = 4 * (max_bias - 1) * diff + (2 - max_bias);

        // Split torque to the slow and fast wheels.
        double alpha = bias / (1 + bias);
        double slow = alpha * torque;
        double fast = torque - slow;

        if (std::abs(speed_rear) < std::abs(speed_front)) {
            torque_rear = slow;
            torque_front = fast;
        } else {
            torque_rear = fast;
            torque_front = slow;
        }
    } else {
        if (whichWheels) {  // whichWheels = 1 is rear wheel driven
            torque_rear = torque;
            torque_front = 0;
        } else {
            torque_rear = 0;
            torque_front = torque;
        }
    }
}

void d11::computeTireRHS(TMeasyState& t_states,
                         const TMeasyParam& t_params,
                         const VehicleParam& v_params,
                         double steering) {
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    // Get the whichTire based variables out of the way
    double fz = t_states._fz;    // vertical force
    double vsy = t_states._vsy;  // y slip velocity
    double vsx = t_states._vsx;  // x slip velocity

    // If the wheel is off contact, set all states to 0 and return
    if (fz <= 0) {
        t_states._xe = 0;
        t_states._ye = 0;
        t_states._omega = 0;
        t_states._xt = 0;
        t_states._rStat = t_params._r0;
        t_states._fx = 0;
        t_states._fy = 0;
        t_states._My = 0;
        return;
    }

    // get our tire deflections so that we can get the loaded radius
    t_states._xt = fz / t_params._kt;
    t_states._rStat = t_params._r0 - t_states._xt;

    double r_eff;
    double rdynco;
    if (fz <= t_params._fzRdynco) {
        rdynco = InterpL(fz, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
        r_eff = rdynco * t_params._r0 + (1. - rdynco) * t_states._rStat;
    } else {
        rdynco = t_params._rdyncoCrit;
        r_eff = rdynco * t_params._r0 + (1. - rdynco) * t_states._rStat;
    }
    // with this r_eff, we can finalize the x slip velocity
    vsx = vsx - (t_states._omega * r_eff);

    // get the transport velocity - 0.01 here is to prevent singularity
    double vta = r_eff * std::abs(t_states._omega) + 0.01;

    // evaluate the slips
    double sx = -vsx / vta;
    double alpha;
    // only front wheel steering
    alpha = std::atan2(vsy, vta) - delta;
    double sy = -std::tan(alpha);

    // limit fz
    if (fz > t_params._pnmax) {
        fz = t_params._pnmax;
    }

    // calculate all curve parameters through interpolation
    double dfx0 = InterpQ(fz, t_params._dfx0Pn, t_params._dfx0P2n, t_params._pn);
    double dfy0 = InterpQ(fz, t_params._dfy0Pn, t_params._dfy0P2n, t_params._pn);

    double fxm = InterpQ(fz, t_params._fxmPn, t_params._fxmP2n, t_params._pn);
    double fym = InterpQ(fz, t_params._fymPn, t_params._fymP2n, t_params._pn);

    double fxs = InterpQ(fz, t_params._fxsPn, t_params._fxsP2n, t_params._pn);
    double fys = InterpQ(fz, t_params._fysPn, t_params._fysP2n, t_params._pn);

    double sxm = InterpL(fz, t_params._sxmPn, t_params._sxmP2n, t_params._pn);
    double sym = InterpL(fz, t_params._symPn, t_params._symP2n, t_params._pn);

    double sxs = InterpL(fz, t_params._sxsPn, t_params._sxsP2n, t_params._pn);
    double sys = InterpL(fz, t_params._sysPn, t_params._sysP2n, t_params._pn);

    // slip normalizing factors
    double hsxn = sxm / (sxm + sym) + (fxm / dfx0) / (fxm / dfx0 + fym / dfy0);
    double hsyn = sym / (sxm + sym) + (fym / dfy0) / (fxm / dfx0 + fym / dfy0);

    // normalized slip
    double sxn = sx / hsxn;
    double syn = sy / hsyn;

    // combined slip
    double sc = std::hypot(sxn, syn);

    // cos and sine alphs
    double calpha;
    double salpha;
    if (sc > 0) {
        calpha = sxn / sc;
        salpha = syn / sc;
    } else {
        calpha = std::sqrt(2.) / 2.;
        salpha = std::sqrt(2.) / 2.;
    }

    // resultant curve parameters in both directions
    double df0 = std::hypot(dfx0 * calpha * hsxn, dfy0 * salpha * hsyn);
    double fm = std::hypot(fxm * calpha, fym * salpha);
    double sm = std::hypot(sxm * calpha / hsxn, sym * salpha / hsyn);
    double fs = std::hypot(fxs * calpha, fys * salpha);
    double ss = std::hypot(sxs * calpha / hsxn, sys * salpha / hsyn);

    // calculate force and force /slip from the curve characteritics
    double f, fos;
    tmxy_combined(f, fos, sc, df0, sm, fm, ss, fs);

    // static or "structural" force
    double Fx, Fy;
    if (sc > 0.) {
        Fx = f * sx / sc;
        Fy = f * sy / sc;
    } else {
        Fx = 0.;
        Fy = 0.;
    }

    // rolling resistance with smoothing
    double vx_min = 0.;
    double vx_max = 0.;

    t_states._My = -sineStep(vta, vx_min, 0., vx_max, 1.) * t_params._rr * fz * t_states._rStat * sgn(t_states._omega);

    // some normalised slip velocities
    double vtxs = r_eff * std::abs(t_states._omega) * hsxn + 0.01;
    double vtys = r_eff * std::abs(t_states._omega) * hsyn + 0.01;

    // double vtxs = r_eff * std::abs(t_states._omega) * hsxn;
    // double vtys = r_eff * std::abs(t_states._omega) * hsyn;

    // Tire velocities for the RHS
    t_states._xedot = (-vtxs * t_params._cx * t_states._xe - fos * vsx) / (vtxs * t_params._dx + fos);
    t_states._yedot = (-vtys * t_params._cy * t_states._ye - fos * (-sy * vta)) / (vtys * t_params._dy + fos);

    // Also compute the tire forces that need to be applied on the vehicle
    // Note here that these tire forces are computed using the current pre
    // integreated xe, unline in the semi-implicit solver
    double fxdyn = t_params._dx * (-vtxs * t_params._cx * t_states._xe - fos * vsx) / (vtxs * t_params._dx + fos) +
                   t_params._cx * t_states._xe;

    double fydyn =
        t_params._dy * ((-vtys * t_params._cy * t_states._ye - fos * (-sy * vta)) / (vtys * t_params._dy + fos)) +
        (t_params._cy * t_states._ye);

    double fxstr =
        clamp(t_states._xe * t_params._cx + t_states._xedot * t_params._dx, -t_params._fxmP2n, t_params._fxmP2n);
    double fystr =
        clamp(t_states._ye * t_params._cy + t_states._yedot * t_params._dy, -t_params._fymP2n, t_params._fymP2n);

    double weightx = sineStep(std::abs(vsx), 1., 1., 1.5, 0.);
    double weighty = sineStep(std::abs(-sy * vta), 1., 1., 1.5, 0.);

    // now finally get the resultant force
    t_states._fx = weightx * fxstr + (1. - weightx) * fxdyn;
    t_states._fy = weighty * fystr + (1. - weighty) * fydyn;
}

void d11::computeTireRHS(TMeasyNrState& t_states,
                         const TMeasyNrParam& t_params,
                         const VehicleParam& v_params,
                         double steering) {
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    // Get the whichTire based variables out of the way
    double fz = t_states._fz;    // vertical force
    double vsy = t_states._vsy;  // y slip velocity
    double vsx = t_states._vsx;  // x slip velocity

    // If the wheel is off contact, set all states to 0 and return
    if (fz <= 0) {
        t_states._omega = 0;
        t_states._xt = 0;
        t_states._rStat = t_params._r0;
        t_states._fx = 0;
        t_states._fy = 0;
        t_states._My = 0;
        return;
    }
    // get our tire deflections so that we can get the loaded radius
    t_states._xt = fz / t_params._kt;
    t_states._rStat = t_params._r0 - t_states._xt;

    // double r_eff = (2 * t_params._r0 + t_states._rStat) / 3.;
    double r_eff;
    double rdynco;
    if (fz <= t_params._fzRdynco) {
        rdynco = InterpL(fz, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
        r_eff = rdynco * t_params._r0 + (1. - rdynco) * t_states._rStat;
    } else {
        rdynco = t_params._rdyncoCrit;
        r_eff = rdynco * t_params._r0 + (1. - rdynco) * t_states._rStat;
    }

    // with this r_eff, we can finalize the x slip velocity
    vsx = vsx - (t_states._omega * r_eff);

    // get the transport velocity - 0.01 here is to prevent singularity
    double vta = r_eff * std::abs(t_states._omega) + 0.01;

    // Compute the combined column force (used for low speed stability)
    double Fx0 = 0;
    double Fy0 = 0;
    computeCombinedColumbForce(Fx0, Fy0, t_params._mu, vsx, vsy, fz, t_params._vcoulomb);

    // evaluate the slips
    double sx = -vsx / vta;
    double alpha;
    // only front wheel steering
    alpha = std::atan2(vsy, vta) - delta;
    double sy = -std::tan(alpha);

    // limit fz
    if (fz > t_params._pnmax) {
        fz = t_params._pnmax;
    }

    // calculate all curve parameters through interpolation
    double dfx0 = InterpQ(fz, t_params._dfx0Pn, t_params._dfx0P2n, t_params._pn);
    double dfy0 = InterpQ(fz, t_params._dfy0Pn, t_params._dfy0P2n, t_params._pn);

    double fxm = InterpQ(fz, t_params._fxmPn, t_params._fxmP2n, t_params._pn);
    double fym = InterpQ(fz, t_params._fymPn, t_params._fymP2n, t_params._pn);

    double fxs = InterpQ(fz, t_params._fxsPn, t_params._fxsP2n, t_params._pn);
    double fys = InterpQ(fz, t_params._fysPn, t_params._fysP2n, t_params._pn);

    double sxm = InterpL(fz, t_params._sxmPn, t_params._sxmP2n, t_params._pn);
    double sym = InterpL(fz, t_params._symPn, t_params._symP2n, t_params._pn);

    double sxs = InterpL(fz, t_params._sxsPn, t_params._sxsP2n, t_params._pn);
    double sys = InterpL(fz, t_params._sysPn, t_params._sysP2n, t_params._pn);

    // Compute the coefficient to "blend" the columb force with the slip force
    double frblend = sineStep(std::abs(t_states._vsx), t_params._frblend_begin, 0., t_params._frblend_end, 1.);
    // Now standard TMeasy tire forces
    // For now, not using any normalization of the slips - similar to Chrono implementation
    double sc = std::hypot(sx, sy);
    double salpha = 0;
    double calpha = 0;
    if (sc > 0) {
        calpha = sx / sc;
        salpha = sy / sc;
    } else {
        calpha = std::sqrt(2.) / 2.;
        salpha = std::sqrt(2.) / 2.;
    }

    // resultant curve parameters in both directions
    double df0 = std::hypot(dfx0 * calpha, dfy0 * salpha);
    double fm = t_params._mu * std::hypot(fxm * calpha, fym * salpha);
    double sm = t_params._mu * std::hypot(sxm * calpha, sym * salpha);
    double fs = t_params._mu * std::hypot(fxs * calpha, fys * salpha);
    double ss = t_params._mu * std::hypot(sxs * calpha, sys * salpha);

    double f, fos;
    tmxy_combined(f, fos, sc, df0, sm, fm, ss, fs);

    // static or "structural" force
    double Fx, Fy;
    if (sc > 0.) {
        Fx = f * sx / sc;
        Fy = f * sy / sc;
    } else {
        Fx = 0.;
        Fy = 0.;
    }

    // Now that we have both the columb force and the slip force, we can combine them with the sine blend to prevent
    // force jumping

    Fx = (1.0 - frblend) * Fx0 + frblend * Fx;
    Fy = (1.0 - frblend) * Fy0 + frblend * Fy;

    // rolling resistance with smoothing -> Below we don't add any smoothing
    double vx_min = 0.;
    double vx_max = 0.;

    t_states._My = -t_params._rr * fz * t_states._rStat * tanh(t_states._omega);

    // Set the tire forces
    t_states._fx = Fx;
    t_states._fy = Fy;
}

void d11::computePowertrainRHS(VehicleState& v_states,
                               TMeasyState& tiref_st,
                               TMeasyState& tirer_st,
                               const VehicleParam& v_params,
                               const TMeasyParam& t_params,
                               const DriverInput& controls) {
    // some variables needed outside
    double torque_t = 0;
    double max_bias = 2;

    // If we have a torque converter
    if (v_params._tcbool) {
        // set reverse flow to false at each timestep
        bool tc_reverse_flow = false;
        // Split the angular velocities all the way uptill the gear box. All
        // from previous time step
        double omega_t = 0.5 * (tiref_st._omega + tirer_st._omega);

        // get the angular velocity at the torque converter wheel side
        // Note, the gear includes the differential gear as well
        double omega_out = omega_t / (v_params._gearRatios[v_states._current_gr]);

        // Get the omega input to the torque from the engine from the previous
        // time step
        double omega_in = v_states._crankOmega;

        double sr, cf, tr;
        if ((omega_out < 1e-9) || (omega_in < 1e-9)) {  // if we are at the start things can get unstable
            sr = 0;
            // Get capacity factor from capacity lookup table
            cf = getMapY(v_params._CFmap, sr);

            // Get torque ratio from Torque ratio lookup table
            tr = getMapY(v_params._TRmap, sr);
        } else {
            // speed ratio for torque converter
            sr = omega_out / omega_in;

            // Check reverse flow
            if (sr > 1.) {
                sr = 1. - (sr - 1.);
                tc_reverse_flow = true;
            }

            if (sr < 0) {
                sr = 0;
            }

            // get capacity factor from lookup table
            cf = getMapY(v_params._CFmap, sr);

            // Get torque ratio from Torque ratio lookup table
            tr = getMapY(v_params._TRmap, sr);
        }
        // torque applied to the crank shaft
        double torque_in = -std::pow((omega_in / cf), 2);

        // if its reverse flow, this should act as a brake
        if (tc_reverse_flow) {
            torque_in = -torque_in;
        }

        // torque applied to the shaft from torque converter on the wheel side
        double torque_out;
        if (tc_reverse_flow) {
            torque_out = -torque_in;
        } else {
            torque_out = -tr * torque_in;
        }

        // Now torque after the transimission
        torque_t = torque_out / v_params._gearRatios[v_states._current_gr];
        if (std::abs((v_states._u - 0) < 1e-9) && (torque_t < 0)) {
            torque_t = 0;
        }

        //////// Integrate Crank shaft
        v_states._dOmega_crank = (1. / v_params._crankInertia) *
                                 (driveTorque(v_params, controls.m_throttle, v_states._crankOmega) + torque_in);

        ////// Gear shift for the next time step -> Here we have to check the
        /// RPM of
        /// the shaft from the T.C
        if (omega_out > v_params._shiftMap[v_states._current_gr]._y) {
            // check if we have enough gears to upshift
            if (v_states._current_gr < v_params._gearRatios.size() - 1) {
                v_states._current_gr++;
            }
        }
        // downshift
        else if (omega_out < v_params._shiftMap[v_states._current_gr]._x) {
            // check if we can down shift
            if (v_states._current_gr > 0) {
                v_states._current_gr--;
            }
        }
    } else {  // If there is no torque converter, then the crank shaft does not
              // have a state so we directly updated the crankOmega to be used
        v_states._crankOmega = 0.5 * (tiref_st._omega + tirer_st._omega) / v_params._gearRatios[v_states._current_gr];

        // The torque after tranny will then just become as there is no torque
        // converter
        torque_t = driveTorque(v_params, controls.m_throttle, v_states._crankOmega) /
                   v_params._gearRatios[v_states._current_gr];

        if (std::abs((v_states._u - 0) < 1e-9) && (torque_t < 0)) {
            torque_t = 0;
        }

        ///// Upshift gear for next time step -> Here the crank shaft is
        /// directly
        /// connected to the gear box
        if (v_states._crankOmega > v_params._shiftMap[v_states._current_gr]._y) {
            // check if we have enough gears to upshift
            if (v_states._current_gr < v_params._gearRatios.size() - 1) {
                v_states._current_gr++;
            }
        }
        // downshift
        else if (v_states._crankOmega < v_params._shiftMap[v_states._current_gr]._x) {
            // check if we can down shift
            if (v_states._current_gr > 0) {
                v_states._current_gr--;
            }
        }
    }

    //////// Amount of torque transmitted to the wheels

    differentialSplit(torque_t, max_bias, tirer_st._omega, tiref_st._omega, tirer_st._engTor, tiref_st._engTor,
                      v_params._driveType, v_params._whichWheels);

    // now use this force for our omegas
    // Get dOmega for each tire
    tiref_st._dOmega = (1 / t_params._jw) * (tiref_st._engTor + tiref_st._My -
                                             sgn(tiref_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                             tiref_st._fx * tiref_st._rStat);

    tirer_st._dOmega = (1 / t_params._jw) * (tirer_st._engTor + tirer_st._My -
                                             sgn(tirer_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                             tirer_st._fx * tirer_st._rStat);
}

void d11::computePowertrainRHS(VehicleState& v_states,
                               TMeasyNrState& tiref_st,
                               TMeasyNrState& tirer_st,
                               const VehicleParam& v_params,
                               const TMeasyNrParam& t_params,
                               const DriverInput& controls) {
    // some variables needed outside
    double torque_t = 0;
    double max_bias = 2;

    // If we have a torque converter
    if (v_params._tcbool) {
        // set reverse flow to false at each timestep
        bool tc_reverse_flow = false;
        // Split the angular velocities all the way uptill the gear box. All
        // from previous time step
        double omega_t = 0.5 * (tiref_st._omega + tirer_st._omega);

        // get the angular velocity at the torque converter wheel side
        // Note, the gear includes the differential gear as well
        double omega_out = omega_t / (v_params._gearRatios[v_states._current_gr]);

        // Get the omega input to the torque from the engine from the previous
        // time step
        double omega_in = v_states._crankOmega;

        double sr, cf, tr;
        if ((omega_out < 1e-9) || (omega_in < 1e-9)) {  // if we are at the start things can get unstable
            sr = 0;
            // Get capacity factor from capacity lookup table
            cf = getMapY(v_params._CFmap, sr);

            // Get torque ratio from Torque ratio lookup table
            tr = getMapY(v_params._TRmap, sr);
        } else {
            // speed ratio for torque converter
            sr = omega_out / omega_in;

            // Check reverse flow
            if (sr > 1.) {
                sr = 1. - (sr - 1.);
                tc_reverse_flow = true;
            }

            if (sr < 0) {
                sr = 0;
            }

            // get capacity factor from lookup table
            cf = getMapY(v_params._CFmap, sr);

            // Get torque ratio from Torque ratio lookup table
            tr = getMapY(v_params._TRmap, sr);
        }
        // torque applied to the crank shaft
        double torque_in = -std::pow((omega_in / cf), 2);

        // if its reverse flow, this should act as a brake
        if (tc_reverse_flow) {
            torque_in = -torque_in;
        }

        // torque applied to the shaft from torque converter on the wheel side
        double torque_out;
        if (tc_reverse_flow) {
            torque_out = -torque_in;
        } else {
            torque_out = -tr * torque_in;
        }

        // Now torque after the transimission
        torque_t = torque_out / v_params._gearRatios[v_states._current_gr];
        if (std::abs((v_states._u - 0) < 1e-9) && (torque_t < 0)) {
            torque_t = 0;
        }

        //////// Integrate Crank shaft
        v_states._dOmega_crank = (1. / v_params._crankInertia) *
                                 (driveTorque(v_params, controls.m_throttle, v_states._crankOmega) + torque_in);

        ////// Gear shift for the next time step -> Here we have to check the
        /// RPM of
        /// the shaft from the T.C
        if (omega_out > v_params._shiftMap[v_states._current_gr]._y) {
            // check if we have enough gears to upshift
            if (v_states._current_gr < v_params._gearRatios.size() - 1) {
                v_states._current_gr++;
            }
        }
        // downshift
        else if (omega_out < v_params._shiftMap[v_states._current_gr]._x) {
            // check if we can down shift
            if (v_states._current_gr > 0) {
                v_states._current_gr--;
            }
        }
    } else {  // If there is no torque converter, then the crank shaft does not
              // have a state so we directly updated the crankOmega to be used
        v_states._crankOmega = 0.5 * (tiref_st._omega + tirer_st._omega) / v_params._gearRatios[v_states._current_gr];

        // The torque after tranny will then just become as there is no torque
        // converter
        torque_t = driveTorque(v_params, controls.m_throttle, v_states._crankOmega) /
                   v_params._gearRatios[v_states._current_gr];

        if (std::abs((v_states._u - 0) < 1e-9) && (torque_t < 0)) {
            torque_t = 0;
        }

        ///// Upshift gear for next time step -> Here the crank shaft is
        /// directly
        /// connected to the gear box
        if (v_states._crankOmega > v_params._shiftMap[v_states._current_gr]._y) {
            // check if we have enough gears to upshift
            if (v_states._current_gr < v_params._gearRatios.size() - 1) {
                v_states._current_gr++;
            }
        }
        // downshift
        else if (v_states._crankOmega < v_params._shiftMap[v_states._current_gr]._x) {
            // check if we can down shift
            if (v_states._current_gr > 0) {
                v_states._current_gr--;
            }
        }
    }

    //////// Amount of torque transmitted to the wheels

    differentialSplit(torque_t, max_bias, tirer_st._omega, tiref_st._omega, tirer_st._engTor, tiref_st._engTor,
                      v_params._driveType, v_params._whichWheels);

    // now use this force for our omegas
    // Get dOmega for each tire
    tiref_st._dOmega = (1 / t_params._jw) * (tiref_st._engTor + tiref_st._My -
                                             sgn(tiref_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                             tiref_st._fx * tiref_st._rStat);

    tirer_st._dOmega = (1 / t_params._jw) * (tirer_st._engTor + tirer_st._My -
                                             sgn(tirer_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                             tirer_st._fx * tirer_st._rStat);
}

void d11::computeVehRHS(VehicleState& v_states,
                        const VehicleParam& v_params,
                        const std::vector<double>& fx,
                        const std::vector<double>& fy) {
    double mt = v_params._m + 2 * (v_params._muf + v_params._mur);

    // ODEs
    v_states._vdot = -v_states._u * v_states._wz + (fy[0] + fy[1]) / mt;
    v_states._udot = v_states._v * v_states._wz + (fx[0] + fx[1]) / mt;
    v_states._wzdot = (v_params._a * fy[0] - v_params._b * fy[1]) / v_params._jz;

    v_states._dx = v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi);
    v_states._dy = v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi);
}

// ---------------------------------------------------------
// Setting the tire and vehicle parameters from the JSON file
// ---------------------------------------------------------

// setting Vehicle parameters using a JSON file
void d11::setVehParamsJSON(VehicleParam& v_params, const char* fileName) {
    // Open the file
    FILE* fp = fopen(fileName, "r");

    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    // parse the stream into DOM tree
    rapidjson::Document d;
    d.ParseStream(is);
    fclose(fp);

    if (d.HasParseError()) {
        std::cout << "Error with rapidjson:" << std::endl << d.GetParseError() << std::endl;
    }

    // the file should have all these parameters defined
    v_params._a = d["a"].GetDouble();
    v_params._b = d["b"].GetDouble();
    v_params._m = d["m"].GetDouble();
    v_params._h = d["h"].GetDouble();
    v_params._jz = d["jz"].GetDouble();
    v_params._muf = d["muf"].GetDouble();
    v_params._mur = d["mur"].GetDouble();

    // Non linear steering which maps the normalized steering input to wheel
    // angle
    v_params._nonLinearSteer = d["nonLinearSteer"].GetBool();
    if (v_params._nonLinearSteer) {
        unsigned int steerMapSize = d["steerMap"].Size();
        for (unsigned int i = 0; i < steerMapSize; i++) {
            MapEntry m;
            m._x = d["steerMap"][i][0u].GetDouble();
            m._y = d["steerMap"][i][1u].GetDouble();
            v_params._steerMap.push_back(m);
        }
    }
    // If there is no non linear steer then the normalized steering input is
    // just multiplied by the max steering wheel angle
    else {
        v_params._maxSteer = d["maxSteer"].GetDouble();
    }

    v_params._maxSteer = d["maxSteer"].GetDouble();

    // read the gear ratios
    unsigned int gears = d["gearRatios"].Size();
    for (unsigned int i = 0; i < gears; i++) {
        v_params._gearRatios.push_back(d["gearRatios"][i].GetDouble());
    }

    // Check if we have upshiftRPM and downshiftRPM set
    if (d.HasMember("upshiftRPM") && d.HasMember("downshiftRPM")) {
        // If this is the case then just set the shift map for all gears to the
        // same value
        for (unsigned int i = 0; i < gears; i++) {
            MapEntry m;
            m._x = d["downshiftRPM"].GetDouble() * rpm2rad;
            m._y = d["upshiftRPM"].GetDouble() * rpm2rad;
            v_params._shiftMap.push_back(m);
        }
    } else {
        // If there is no upshiftRPM and downshiftRPM then there should be a
        // shift map and this should have as many entries as the number of gears
        for (unsigned int i = 0; i < gears; i++) {
            MapEntry m;
            m._x = d["shiftMap"][i][0u].GetDouble() * rpm2rad;  // downshift
            m._y = d["shiftMap"][i][1u].GetDouble() * rpm2rad;  // upshift
            v_params._shiftMap.push_back(m);
        }
    }

    v_params._tcbool = d["tcBool"].GetBool();

    v_params._maxBrakeTorque = d["maxBrakeTorque"].GetDouble();
    v_params._step = d["step"].GetDouble();

    v_params._throttleMod = d["throttleMod"].GetBool();
    // Read the powertrain map
    unsigned int map_size = d["torqueMap"].Size();
    for (unsigned int i = 0; i < map_size; i++) {
        MapEntry m;
        m._x = d["torqueMap"][i][0u].GetDouble() * rpm2rad;
        m._y = d["torqueMap"][i][1u].GetDouble();
        v_params._powertrainMap.push_back(m);
    }

    // Read the losses map
    unsigned int map_size2 = d["lossesMap"].Size();
    for (unsigned int i = 0; i < map_size2; i++) {
        MapEntry m;
        m._x = d["lossesMap"][i][0u].GetDouble() * rpm2rad;
        m._y = d["lossesMap"][i][1u].GetDouble();
        v_params._lossesMap.push_back(m);
    }

    // if we have a torque converter, we need this data
    if (v_params._tcbool) {
        v_params._crankInertia = d["crankInertia"].GetDouble();
        unsigned int map_size_cf = d["capacityFactorMap"].Size();
        for (unsigned int i = 0; i < map_size_cf; i++) {
            MapEntry m;
            m._x = d["capacityFactorMap"][i][0u].GetDouble();
            m._y = d["capacityFactorMap"][i][1u].GetDouble();
            v_params._CFmap.push_back(m);
        }

        unsigned int map_size_tr = d["torqueRatioMap"].Size();
        for (unsigned int i = 0; i < map_size_tr; i++) {
            MapEntry m;
            m._x = d["torqueRatioMap"][i][0u].GetDouble();
            m._y = d["torqueRatioMap"][i][1u].GetDouble();
            v_params._TRmap.push_back(m);
        }
    }

    // Store the drive Type (1 is 4WD and 0 is 2WD (rear wheels))
    if (d.HasMember("4wd")) {
        v_params._driveType = d["4wd"].GetBool();
        v_params._whichWheels = d["rearWheels"].GetBool();  // 1 for rear wheels 0 for front wheels
    }
}
// setting Tire parameters using a JSON file
void d11::setTireParamsJSON(TMeasyParam& t_params, const char* fileName) {
    // Open the file
    FILE* fp = fopen(fileName, "r");

    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    // parse the stream into DOM tree
    rapidjson::Document d;
    d.ParseStream(is);
    fclose(fp);

    if (d.HasParseError()) {
        std::cout << "Error with rapidjson:" << std::endl << d.GetParseError() << std::endl;
    }

    t_params._jw = d["jw"].GetDouble() * 2.;
    t_params._rr = d["rr"].GetDouble() * 2.;
    t_params._r0 = d["r0"].GetDouble();
    t_params._pn = d["pn"].GetDouble() * 2.;
    t_params._pnmax = d["pnmax"].GetDouble() * 2.;
    t_params._cx = d["cx"].GetDouble() * 2.;
    t_params._cy = d["cy"].GetDouble() * 2.;
    t_params._kt = d["kt"].GetDouble() * 2.;
    t_params._dx = d["dx"].GetDouble() * 2.;
    t_params._dy = d["dy"].GetDouble() * 2.;
    t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
    t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
    t_params._fzRdynco = d["fzRdynco"].GetDouble() * 2.;
    t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble() * 2.;

    t_params._dfx0Pn = d["dfx0Pn"].GetDouble() * 2.;
    t_params._dfx0P2n = d["dfx0P2n"].GetDouble() * 2.;
    t_params._fxmPn = d["fxmPn"].GetDouble() * 2.;
    t_params._fxmP2n = d["fxmP2n"].GetDouble() * 2.;
    t_params._fxsPn = d["fxsPn"].GetDouble() * 2.;
    t_params._fxsP2n = d["fxsP2n"].GetDouble() * 2.;
    t_params._sxmPn = d["sxmPn"].GetDouble();
    t_params._sxmP2n = d["sxmP2n"].GetDouble();
    t_params._sxsPn = d["sxsPn"].GetDouble();
    t_params._sxsP2n = d["sxsP2n"].GetDouble();

    t_params._dfy0Pn = d["dfy0Pn"].GetDouble() * 2.;
    t_params._dfy0P2n = d["dfy0P2n"].GetDouble() * 2.;
    t_params._fymPn = d["fymPn"].GetDouble() * 2.;
    t_params._fymP2n = d["fymP2n"].GetDouble() * 2.;
    t_params._fysPn = d["fysPn"].GetDouble() * 2.;
    t_params._fysP2n = d["fysP2n"].GetDouble() * 2.;
    t_params._symPn = d["symPn"].GetDouble();
    t_params._symP2n = d["symP2n"].GetDouble();
    t_params._sysPn = d["sysPn"].GetDouble();
    t_params._sysP2n = d["sysP2n"].GetDouble();

    t_params._step = d["step"].GetDouble();
}

// setting Tire parameters using a JSON file for a TMeasyNr
void d11::setTireParamsJSON(TMeasyNrParam& t_params, const char* fileName) {
    // Open the file
    FILE* fp = fopen(fileName, "r");

    char readBuffer[32768];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    // parse the stream into DOM tree
    rapidjson::Document d;
    d.ParseStream(is);
    fclose(fp);

    if (d.HasParseError()) {
        std::cout << "Error with rapidjson:" << std::endl << d.GetParseError() << std::endl;
    }

    // First check if the user wants to specify high-level tire parameters or low level
    // Check if there is a member called "highLevelParams", if it does not exist, then we assume the user specifies low
    // level parameters
    if (d.HasMember("highLevelParams")) {
        if (d["highLevelParams"].GetBool()) {
            // Set basic desing parameters
            t_params._jw = d["jw"].GetDouble() * 2.;
            t_params._rr = d["rr"].GetDouble() * 2.;
            t_params._mu = d["mu"].GetDouble() * 2.;
            t_params._r0 = d["r0"].GetDouble();
            t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
            t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
            t_params._fzRdynco = d["fzRdynco"].GetDouble() * 2.;
            t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble() * 2.;

            // Some very specific parameters that the user might not want to define, set to default otherwise
            if (d.HasMember("vCoulomb")) {
                t_params._vcoulomb = d["vCoulomb"].GetDouble();
            } else {
                t_params._vcoulomb = 1.0;
            }
            if (d.HasMember("frblendBegin")) {
                t_params._frblend_begin = d["frblendBegin"].GetDouble();
            } else {
                t_params._frblend_begin = 1.0;
            }
            if (d.HasMember("frblendEnd")) {
                t_params._frblend_end = d["frblendEnd"].GetDouble();
            } else {
                t_params._frblend_end = 3.0;
            }

            // Get the high level parameters from the JSON file
            double width = d["width"].GetDouble();
            double rimRadius = d["rimRadius"].GetDouble();
            double p_li = 1.0;   // Inflation pressure at load index [Pa]
            double p_use = 1.0;  // Inflation pressure at use time [Pa]
            // Tire can either be specified with load index or bearing capacity
            if (d.HasMember("loadIndex")) {
                // Information about tire inflation pressure might be present if the user wants to use the inflation
                if (d.HasMember("inflationPressureDesign")) {
                    p_li = d["inflationPressureDesign"].GetDouble();
                } else {
                    p_li = 0.0;
                }
                if (d.HasMember("inflationPressureUse")) {
                    p_use = d["inflationPressureUse"].GetDouble();
                } else {
                    p_use = 0.0;
                }
                if (p_use > 0.0 && p_li > 0.0) {
                    ////pressure_info_found = true;
                } else {
                    p_li = p_use = 1.0;
                }

                unsigned int li = d["loadIndex"].GetUint();
                t_params._li = li;  // Just setting this although its not used anywhere for now
                t_params._p_li = p_li;
                t_params._p_use = p_use;
                std::string vehicle_type = d["vehicleType"].GetString();
                if (vehicle_type.compare("Truck") == 0) {
                    GuessTruck80Par(li, width, (t_params._r0 - rimRadius) / width, 2 * rimRadius, p_li, p_use,
                                    t_params);
                } else {
                    GuessPassCar70Par(li, width, (t_params._r0 - rimRadius) / width, 2 * rimRadius, p_li, p_use,
                                      t_params);
                }
            } else {  // if it does not have load index, it has to have maximum bearing capacity
                      // Information about tire inflation pressure might be present if the user wants to use the
                      // inflation
                if (d.HasMember("inflationPressureDesign")) {
                    p_li = d["inflationPressureDesign"].GetDouble();
                } else {
                    p_li = 0.0;
                }
                if (d.HasMember("inflationPressureUse")) {
                    p_use = d["inflationPressureUse"].GetDouble();
                } else {
                    p_use = 0.0;
                }
                if (p_use > 0.0 && p_li > 0.0) {
                    ////pressure_info_found = true;
                } else {
                    p_li = p_use = 1.0;
                }
                t_params._p_li = p_li;
                t_params._p_use = p_use;
                double bearing_capacity = d["maximumBearingCapacity"].GetDouble();
                t_params._bearingCapacity =
                    bearing_capacity;  // Just setting this so that it can be used to set externally while calibrating
                std::string vehicle_type = d["vehicleType"].GetString();
                if (vehicle_type.compare("Truck") == 0) {
                    GuessTruck80Par(bearing_capacity, width, (t_params._r0 - rimRadius) / width, 2 * rimRadius, p_li,
                                    p_use, t_params);
                } else {
                    GuessPassCar70Par(bearing_capacity, width, (t_params._r0 - rimRadius) / width, 2 * rimRadius, p_li,
                                      p_use, t_params);
                }
            }

        } else {  // If high level params is false, then user should have provided the low level params
            t_params._jw = d["jw"].GetDouble() * 2.;
            t_params._rr = d["rr"].GetDouble() * 2.;
            t_params._r0 = d["r0"].GetDouble();
            t_params._pn = d["pn"].GetDouble() * 2.;
            t_params._pnmax = d["pnmax"].GetDouble() * 2.;
            t_params._kt = d["kt"].GetDouble() * 2.;
            t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
            t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
            t_params._fzRdynco = d["fzRdynco"].GetDouble() * 2.;
            t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble() * 2.;

            t_params._dfx0Pn = d["dfx0Pn"].GetDouble() * 2.;
            t_params._dfx0P2n = d["dfx0P2n"].GetDouble() * 2.;
            t_params._fxmPn = d["fxmPn"].GetDouble() * 2.;
            t_params._fxmP2n = d["fxmP2n"].GetDouble() * 2.;
            t_params._fxsPn = d["fxsPn"].GetDouble() * 2.;
            t_params._fxsP2n = d["fxsP2n"].GetDouble() * 2.;
            t_params._sxmPn = d["sxmPn"].GetDouble();
            t_params._sxmP2n = d["sxmP2n"].GetDouble();
            t_params._sxsPn = d["sxsPn"].GetDouble();
            t_params._sxsP2n = d["sxsP2n"].GetDouble();

            t_params._dfy0Pn = d["dfy0Pn"].GetDouble() * 2.;
            t_params._dfy0P2n = d["dfy0P2n"].GetDouble() * 2.;
            t_params._fymPn = d["fymPn"].GetDouble() * 2.;
            t_params._fymP2n = d["fymP2n"].GetDouble() * 2.;
            t_params._fysPn = d["fysPn"].GetDouble() * 2.;
            t_params._fysP2n = d["fysP2n"].GetDouble() * 2.;
            t_params._symPn = d["symPn"].GetDouble();
            t_params._symP2n = d["symP2n"].GetDouble();
            t_params._sysPn = d["sysPn"].GetDouble();
            t_params._sysP2n = d["sysP2n"].GetDouble();
        }

    } else {
        t_params._jw = d["jw"].GetDouble() * 2.;
        t_params._rr = d["rr"].GetDouble() * 2.;
        t_params._r0 = d["r0"].GetDouble();
        t_params._pn = d["pn"].GetDouble() * 2.;
        t_params._pnmax = d["pnmax"].GetDouble() * 2.;
        t_params._kt = d["kt"].GetDouble() * 2.;
        t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
        t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
        t_params._fzRdynco = d["fzRdynco"].GetDouble() * 2.;
        t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble() * 2.;

        t_params._dfx0Pn = d["dfx0Pn"].GetDouble() * 2.;
        t_params._dfx0P2n = d["dfx0P2n"].GetDouble() * 2.;
        t_params._fxmPn = d["fxmPn"].GetDouble() * 2.;
        t_params._fxmP2n = d["fxmP2n"].GetDouble() * 2.;
        t_params._fxsPn = d["fxsPn"].GetDouble() * 2.;
        t_params._fxsP2n = d["fxsP2n"].GetDouble() * 2.;
        t_params._sxmPn = d["sxmPn"].GetDouble();
        t_params._sxmP2n = d["sxmP2n"].GetDouble();
        t_params._sxsPn = d["sxsPn"].GetDouble();
        t_params._sxsP2n = d["sxsP2n"].GetDouble();

        t_params._dfy0Pn = d["dfy0Pn"].GetDouble() * 2.;
        t_params._dfy0P2n = d["dfy0P2n"].GetDouble() * 2.;
        t_params._fymPn = d["fymPn"].GetDouble() * 2.;
        t_params._fymP2n = d["fymP2n"].GetDouble() * 2.;
        t_params._fysPn = d["fysPn"].GetDouble() * 2.;
        t_params._fysP2n = d["fysP2n"].GetDouble() * 2.;
        t_params._symPn = d["symPn"].GetDouble();
        t_params._symP2n = d["symP2n"].GetDouble();
        t_params._sysPn = d["sysPn"].GetDouble();
        t_params._sysP2n = d["sysP2n"].GetDouble();
    }
}

// Utility functions that guess the tire parameters for a TMeasy tire based on standard tire specifications that user
// can get from a spec sheet These functions are directly copy pasted from Chrono with minor modifications

// Function to compute the max tire load from the load index specified by the user
double d11::GetTireMaxLoad(unsigned int li) {
    double Weight_per_Tire[] = {
        45,    46.5,  47.5,   48.7,   50,     51.5,   53,     54.5,   56,     58,     60,     61.5,   63,     65,
        67,    69,    71,     73,     75,     77.5,   80.0,   82.5,   85.0,   87.5,   90.0,   92.5,   95.0,   97.5,
        100.0, 103,   106,    109,    112,    115,    118,    121,    125,    128,    132,    136,    140,    145,
        150,   155,   160,    165,    170,    175,    180,    185,    190,    195,    200,    206,    212,    218,
        224,   230,   236,    243,    250,    257,    265,    272,    280,    290,    300,    307,    315,    325,
        335,   345,   355,    365,    375,    387,    400,    412,    425,    437,    450,    462,    475,    487,
        500,   515,   530,    545,    560,    580,    600,    615,    630,    650,    670,    690,    710,    730,
        750,   775,   800,    825,    850,    875,    900,    925,    950,    975,    1000,   1030,   1060,   1090,
        1120,  1150,  1180,   1215,   1250,   1285,   1320,   1360,   1400,   1450,   1500,   1550,   1600,   1650,
        1700,  1750,  1800,   1850,   1900,   1950,   2000,   2060,   2120,   2180,   2240,   2300,   2360,   2430,
        2500,  2575,  2650,   2725,   2800,   2900,   3000,   3075,   3150,   3250,   3350,   3450,   3550,   3650,
        3750,  3875,  4000,   4125,   4250,   4375,   4500,   4625,   4750,   4875,   5000,   5150,   5300,   5450,
        5600,  5850,  6000,   6150,   6300,   6500,   6700,   6900,   7100,   7300,   7500,   7750,   8000,   8250,
        8500,  8750,  9000,   9250,   9500,   9750,   10000,  10300,  10600,  10900,  11200,  11500,  11800,  12150,
        12500, 12850, 13200,  13600,  14000,  14500,  15000,  15550,  16000,  16500,  17000,  17500,  18000,  18500,
        19000, 19500, 20000,  20600,  21200,  21800,  22400,  23000,  23600,  24300,  25000,  25750,  26500,  27250,
        28000, 29000, 30000,  30750,  31500,  32500,  33500,  34500,  35500,  36500,  37500,  38750,  40000,  41250,
        42500, 43750, 45000,  46250,  47500,  48750,  50000,  51500,  53000,  54500,  56000,  58000,  60000,  61500,
        63000, 65000, 67000,  69000,  71000,  73000,  75000,  77500,  80000,  82500,  85000,  87500,  90000,  92500,
        95000, 97500, 100000, 103000, 106000, 109000, 112000, 115000, 118000, 121000, 125000, 128500, 132000, 136000};

    unsigned int nw = sizeof(Weight_per_Tire) / sizeof(double);
    const double g = 9.81;
    double fmax;
    if (li < nw) {
        fmax = Weight_per_Tire[li] * g;
    } else {
        fmax = Weight_per_Tire[nw - 1] * g;
    }
    return fmax;
}

// Guessing tire parameters for a truck tire
void d11::GuessTruck80Par(unsigned int li,          // tire load index
                          double tireWidth,         // [m]
                          double ratio,             // [] = use 0.75 meaning 75%
                          double rimDia,            // rim diameter [m]
                          double pinfl_li,          // inflation pressure at load index
                          double pinfl_use,         // inflation pressure in this configuration
                          TMeasyNrParam& t_params)  // damping ratio
{
    double tireLoad = GetTireMaxLoad(li);
    GuessTruck80Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use, t_params);
}
void d11::GuessTruck80Par(double tireLoad,   // tire load index
                          double tireWidth,  // [m]
                          double ratio,      // [] = use 0.75 meaning 75%
                          double rimDia,     // rim diameter [m]
                          double pinfl_li,   // inflation pressure at load index
                          double pinfl_use,  // inflation pressure in this configuration
                          TMeasyNrParam& t_params) {
    // damping ratio{
    double secth = tireWidth * ratio;  // tire section height
    double defl_max = 0.16 * secth;    // deflection at tire payload

    t_params._pn = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);
    t_params._pnmax = 3.5 * t_params._pn;

    double CZ = tireLoad / defl_max;

    t_params._kt = CZ;  // Set the tire vertical stiffness

    t_params._rim_radius = 0.5 * rimDia;

    t_params._width = tireWidth;

    // Normalized Parameters gained from data set containing original data from Pacejka book
    t_params._dfx0Pn = 17.7764 * t_params._pn * 2.;
    t_params._dfx0P2n = 14.5301 * 2.0 * t_params._pn * 2.;
    t_params._fxmPn = 0.89965 * t_params._pn * 2.;
    t_params._fxmP2n = 0.77751 * 2.0 * t_params._pn * 2.;
    t_params._fxsPn = 0.46183 * t_params._pn * 2.;
    t_params._fxsP2n = 0.42349 * 2.0 * t_params._pn * 2.;
    t_params._sxmPn = 0.10811;
    t_params._sxmP2n = 0.12389;
    t_params._sxsPn = 0.66667;
    t_params._sxsP2n = 0.66667;
    t_params._dfy0Pn = 7.4013 * t_params._pn * 2.;
    t_params._dfy0P2n = 6.8505 * 2.0 * t_params._pn * 2.;
    t_params._fymPn = 0.75876 * t_params._pn * 2.;
    t_params._fymP2n = 0.72628 * 2.0 * t_params._pn * 2.;
    t_params._fysPn = 0.68276 * t_params._pn * 2.;
    t_params._fysP2n = 0.65319 * 2.0 * t_params._pn * 2.;
    t_params._symPn = 0.33167;
    t_params._symP2n = 0.33216;
    t_params._sysPn = 1.0296;
    t_params._sysP2n = 1.0296;
}

// Guessing tire parameters for a passenger car
void d11::GuessPassCar70Par(unsigned int li,          // tire load index
                            double tireWidth,         // [m]
                            double ratio,             // [] = use 0.75 meaning 75%
                            double rimDia,            // rim diameter [m]
                            double pinfl_li,          // inflation pressure at load index
                            double pinfl_use,         // inflation pressure in this configuration
                            TMeasyNrParam& t_params)  // damping ratio
{
    double tireLoad = GetTireMaxLoad(li);
    GuessPassCar70Par(tireLoad, tireWidth, ratio, rimDia, pinfl_li, pinfl_use, t_params);
}
void d11::GuessPassCar70Par(double tireLoad,            // tire load index
                            double tireWidth,           // [m]
                            double ratio,               // [] = use 0.75 meaning 75%
                            double rimDia,              // rim diameter [m]
                            double pinfl_li,            // inflation pressure at load index
                            double pinfl_use,           // inflation pressure in this configuration
                            TMeasyNrParam& t_params) {  // damping ratio
    double secth = tireWidth * ratio;                   // tire section height
    double defl_max = 0.16 * secth;                     // deflection at tire payload

    t_params._pn = 0.5 * tireLoad * pow(pinfl_use / pinfl_li, 0.8);
    t_params._pnmax = 3.5 * t_params._pn;

    double CZ = tireLoad / defl_max;

    t_params._kt = CZ;  // Set the tire vertical stiffness

    t_params._rim_radius = 0.5 * rimDia;

    t_params._width = tireWidth;

    // Normalized Parameters gained from data set containing original data from Pacejka book
    t_params._dfx0Pn = 18.3741 * t_params._pn * 2.;
    t_params._dfx0P2n = 19.4669 * 2.0 * t_params._pn * 2.;
    t_params._fxmPn = 1.1292 * t_params._pn * 2.;
    t_params._fxmP2n = 1.0896 * 2.0 * t_params._pn * 2.;
    t_params._fxsPn = 0.80149 * t_params._pn * 2.;
    t_params._fxsP2n = 0.76917 * 2.0 * t_params._pn * 2.;
    t_params._sxmPn = 0.13913;
    t_params._sxmP2n = 0.13913;
    t_params._sxsPn = 0.66667;
    t_params._sxsP2n = 0.66667;
    t_params._dfy0Pn = 15.9826 * t_params._pn * 2.;
    t_params._dfy0P2n = 12.8509 * 2.0 * t_params._pn * 2.;
    t_params._fymPn = 1.0009 * t_params._pn * 2.;
    t_params._fymP2n = 0.91367 * 2.0 * t_params._pn * 2.;
    t_params._fysPn = 0.8336 * t_params._pn * 2.;
    t_params._fysP2n = 0.77336 * 2.0 * t_params._pn * 2.;
    t_params._symPn = 0.14852;
    t_params._symP2n = 0.18504;
    t_params._sysPn = 0.96524;
    t_params._sysP2n = 1.0714;
}
