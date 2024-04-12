#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof24.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace d24;

// -----------------------------------------------------------------------------
// Initialize functions
// -----------------------------------------------------------------------------

void d24::tireInit(TMeasyParam& t_params) {
    // calculates some critical values that are needed
    t_params._fzRdynco = (t_params._pn * (t_params._rdyncoP2n - 2.0 * t_params._rdyncoPn + 1.)) /
                         (2. * (t_params._rdyncoP2n - t_params._rdyncoPn));

    t_params._rdyncoCrit = InterpL(t_params._fzRdynco, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
}

void d24::tireInit(TMeasyNrParam& t_params) {
    // calculates some critical values that are needed
    t_params._fzRdynco = (t_params._pn * (t_params._rdyncoP2n - 2.0 * t_params._rdyncoPn + 1.)) /
                         (2. * (t_params._rdyncoP2n - t_params._rdyncoPn));

    t_params._rdyncoCrit = InterpL(t_params._fzRdynco, t_params._rdyncoPn, t_params._rdyncoP2n, t_params._pn);
}

void d24::initializeTireSus(const VehicleState& v_states,
                            TMeasyState& tirelf_st,
                            TMeasyState& tirerf_st,
                            TMeasyState& tirelr_st,
                            TMeasyState& tirerr_st,
                            SuspensionState& suslf_st,
                            SuspensionState& susrf_st,
                            SuspensionState& suslr_st,
                            SuspensionState& susrr_st,
                            const VehicleParam& v_params,
                            const TMeasyParam& t_params,
                            const SuspensionParam& sus_params) {
    // Compute the tire loads based on static loads at rest
    // Initialize the vertical forces based on the vehicle weight
    double weight_split = ((v_params._m * G * v_params._b) / (2 * (v_params._a + v_params._b)) + v_params._muf * G);
    tirelf_st._fz = tirerf_st._fz = weight_split;

    weight_split = ((v_params._m * G * v_params._a) / (2 * (v_params._a + v_params._b)) + v_params._mur * G);

    tirelr_st._fz = tirerr_st._fz = weight_split;

    // Initialize the tire compression
    tirelf_st._xt = tirelf_st._fz / t_params._kt;
    tirerf_st._xt = tirerf_st._fz / t_params._kt;
    tirelr_st._xt = tirelr_st._fz / t_params._kt;
    tirerr_st._xt = tirerr_st._fz / t_params._kt;

    // Initialize spring compression
    suslf_st._xsi = (v_params._m * G * v_params._b) / (2 * (v_params._a + v_params._b) * sus_params._ks);
    suslr_st._xsi = (v_params._m * G * v_params._a) / (2 * (v_params._a + v_params._b) * sus_params._ks);
    susrf_st._xsi = suslf_st._xsi;
    susrr_st._xsi = suslr_st._xsi;

    suslf_st._xs = suslf_st._xsi;
    susrf_st._xs = susrf_st._xsi;
    suslr_st._xs = suslr_st._xsi;
    susrr_st._xs = susrr_st._xsi;

    // Initialize the length of the strut
    suslf_st._lsi = v_params._h - (t_params._r0 - tirelf_st._xt);
    susrf_st._lsi = v_params._h - (t_params._r0 - tirerf_st._xt);
    suslr_st._lsi = v_params._h - (t_params._r0 - tirelr_st._xt);
    susrr_st._lsi = v_params._h - (t_params._r0 - tirerr_st._xt);

    suslf_st._ls = suslf_st._lsi;
    susrf_st._ls = susrf_st._lsi;
    suslr_st._ls = suslr_st._lsi;
    susrr_st._ls = susrr_st._lsi;

    // z direction
    suslf_st._wu = v_states._w;
    susrf_st._wu = v_states._w;
    suslr_st._wu = v_states._w;
    susrr_st._wu = v_states._w;
}

void d24::initializeTireSus(const VehicleState& v_states,
                            TMeasyNrState& tirelf_st,
                            TMeasyNrState& tirerf_st,
                            TMeasyNrState& tirelr_st,
                            TMeasyNrState& tirerr_st,
                            SuspensionState& suslf_st,
                            SuspensionState& susrf_st,
                            SuspensionState& suslr_st,
                            SuspensionState& susrr_st,
                            const VehicleParam& v_params,
                            const TMeasyNrParam& t_params,
                            const SuspensionParam& sus_params) {
    // Compute the tire loads based on static loads at rest
    // Initialize the vertical forces based on the vehicle weight
    double weight_split = ((v_params._m * G * v_params._b) / (2 * (v_params._a + v_params._b)) + v_params._muf * G);
    tirelf_st._fz = tirerf_st._fz = weight_split;

    weight_split = ((v_params._m * G * v_params._a) / (2 * (v_params._a + v_params._b)) + v_params._mur * G);

    tirelr_st._fz = tirerr_st._fz = weight_split;

    // Initialize the tire compression
    tirelf_st._xt = tirelf_st._fz / t_params._kt;
    tirerf_st._xt = tirerf_st._fz / t_params._kt;
    tirelr_st._xt = tirelr_st._fz / t_params._kt;
    tirerr_st._xt = tirerr_st._fz / t_params._kt;

    // Initialize spring compression
    suslf_st._xsi = (v_params._m * G * v_params._b) / (2 * (v_params._a + v_params._b) * sus_params._ks);
    suslr_st._xsi = (v_params._m * G * v_params._a) / (2 * (v_params._a + v_params._b) * sus_params._ks);
    susrf_st._xsi = suslf_st._xsi;
    susrr_st._xsi = suslr_st._xsi;

    suslf_st._xs = suslf_st._xsi;
    susrf_st._xs = susrf_st._xsi;
    suslr_st._xs = suslr_st._xsi;
    susrr_st._xs = susrr_st._xsi;

    // Initialize the length of the strut
    suslf_st._lsi = v_params._h - (t_params._r0 - tirelf_st._xt);
    susrf_st._lsi = v_params._h - (t_params._r0 - tirerf_st._xt);
    suslr_st._lsi = v_params._h - (t_params._r0 - tirelr_st._xt);
    susrr_st._lsi = v_params._h - (t_params._r0 - tirerr_st._xt);

    suslf_st._ls = suslf_st._lsi;
    susrf_st._ls = susrf_st._lsi;
    suslr_st._ls = suslr_st._lsi;
    susrr_st._ls = susrr_st._lsi;

    // z direction
    suslf_st._wu = v_states._w;
    susrf_st._wu = v_states._w;
    suslr_st._wu = v_states._w;
    susrr_st._wu = v_states._w;
}
// -----------------------------------------------------------------------------
// Frame Transformation functions
// -----------------------------------------------------------------------------
void d24::vehToSusTransform(const VehicleState& v_states,
                            const TMeasyState& tirelf_st,
                            const TMeasyState& tirerf_st,
                            const TMeasyState& tirelr_st,
                            const TMeasyState& tirerr_st,
                            SuspensionState& suslf_st,
                            SuspensionState& susrf_st,
                            SuspensionState& suslr_st,
                            SuspensionState& susrr_st,
                            const VehicleParam& v_params) {
    // Struct velocities transformed from the vehicle velocities
    suslf_st._us = (-v_params._cf * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    suslf_st._vs = v_params._a * v_states._wz + v_states._v;           // y direction
    suslf_st._ws = (v_params._cf * v_states._wx / 2.) - (v_params._a * v_states._wy) + v_states._w;  // z direction

    // RF
    susrf_st._us = (v_params._cf * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    susrf_st._vs = v_params._a * v_states._wz + v_states._v;          // y direction
    susrf_st._ws = (-v_params._cf * v_states._wx / 2.) - (v_params._a * v_states._wy) + v_states._w;  // z direction

    // LR
    suslr_st._us = (-v_params._cr * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    suslr_st._vs = -v_params._b * v_states._wz + v_states._v;          // y direction
    suslr_st._ws = (v_params._cr * v_states._wx / 2.) + v_params._b * v_states._wy + v_states._w;  // z direction

    // RR
    susrr_st._us = (v_params._cr * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    susrr_st._vs = -v_params._b * v_states._wz + v_states._v;         // y direction
    susrr_st._ws = (-v_params._cr * v_states._wx / 2.) + v_params._b * v_states._wy + v_states._w;  // z direction

    // Evaluate the length of the struct and its compression velocity in current time step
    // This is to get the unsprung mass velocity and accelerations in G-RF

    // Instantaneous length of the strut
    suslf_st._ls = suslf_st._lsi - (suslf_st._xs - suslf_st._xsi);
    susrf_st._ls = susrf_st._lsi - (susrf_st._xs - susrf_st._xsi);
    suslr_st._ls = suslr_st._lsi - (suslr_st._xs - suslr_st._xsi);
    susrr_st._ls = susrr_st._lsi - (susrr_st._xs - susrr_st._xsi);

    // Unsprung mass velocities in G-RF
    // x direction
    suslf_st._uu = suslf_st._us - suslf_st._ls * v_states._wy;  // LF
    susrf_st._uu = susrf_st._us - susrf_st._ls * v_states._wy;  // RF
    suslr_st._uu = suslr_st._us - suslr_st._ls * v_states._wy;  // LR
    susrr_st._uu = susrr_st._us - susrr_st._ls * v_states._wy;  // RR

    // y direction
    suslf_st._vu = suslf_st._vs + suslf_st._ls * v_states._wx;  // LF
    susrf_st._vu = susrf_st._vs + susrf_st._ls * v_states._wx;  // RF
    suslr_st._vu = suslr_st._vs + suslr_st._ls * v_states._wx;  // LR
    susrr_st._vu = susrr_st._vs + susrr_st._ls * v_states._wx;  // RR

    // z direction is a differential equation that is solved in suspension advance

    // Strut accelerations from vehicle accelerations
    // LF
    suslf_st._dus = (-v_params._cf * v_states._wzdot) / 2 + v_states._udot;
    suslf_st._dvs = v_params._a * v_states._wzdot + v_states._vdot;
    // RF
    susrf_st._dus = (v_params._cf * v_states._wzdot) / 2 + v_states._udot;
    susrf_st._dvs = v_params._a * v_states._wzdot + v_states._vdot;
    // LR
    suslr_st._dus = (-v_params._cr * v_states._wzdot) / 2 + v_states._udot;
    suslr_st._dvs = -v_params._b * v_states._wzdot + v_states._vdot;
    // RR
    susrr_st._dus = (v_params._cr * v_states._wzdot) / 2 + v_states._udot;
    susrr_st._dvs = (-v_params._b * v_states._wzdot) + v_states._vdot;

    // Unsprung mass acceleration in G-RF
    // RF
    susrf_st._duu = susrf_st._dus - (-susrf_st._dxs * v_states._wy + susrf_st._ls * v_states._wydot);
    susrf_st._dvu = susrf_st._dvs + (-susrf_st._dxs * v_states._wx + susrf_st._ls * v_states._wxdot);
    // LF
    suslf_st._duu = suslf_st._dus - (-suslf_st._dxs * v_states._wy + suslf_st._ls * v_states._wydot);
    suslf_st._dvu = suslf_st._dvs + (-suslf_st._dxs * v_states._wx + suslf_st._ls * v_states._wxdot);
    // LR
    suslr_st._duu = suslr_st._dus - (-suslr_st._dxs * v_states._wy + suslr_st._ls * v_states._wydot);
    suslr_st._dvu = suslr_st._dvs + (-suslr_st._dxs * v_states._wx + suslr_st._ls * v_states._wxdot);
    // RR
    susrr_st._duu = susrr_st._dus - (-susrr_st._dxs * v_states._wy + susrr_st._ls * v_states._wydot);
    susrr_st._dvu = susrr_st._dvs + (-susrr_st._dxs * v_states._wx + susrr_st._ls * v_states._wxdot);
}

void d24::vehToSusTransform(const VehicleState& v_states,
                            const TMeasyNrState& tirelf_st,
                            const TMeasyNrState& tirerf_st,
                            const TMeasyNrState& tirelr_st,
                            const TMeasyNrState& tirerr_st,
                            SuspensionState& suslf_st,
                            SuspensionState& susrf_st,
                            SuspensionState& suslr_st,
                            SuspensionState& susrr_st,
                            const VehicleParam& v_params) {
    // Struct velocities transformed from the vehicle velocities
    suslf_st._us = (-v_params._cf * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    suslf_st._vs = v_params._a * v_states._wz + v_states._v;           // y direction
    suslf_st._ws = (v_params._cf * v_states._wx / 2.) - (v_params._a * v_states._wy) + v_states._w;  // z direction

    // RF
    susrf_st._us = (v_params._cf * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    susrf_st._vs = v_params._a * v_states._wz + v_states._v;          // y direction
    susrf_st._ws = (-v_params._cf * v_states._wx / 2.) - (v_params._a * v_states._wy) + v_states._w;  // z direction

    // LR
    suslr_st._us = (-v_params._cr * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    suslr_st._vs = -v_params._b * v_states._wz + v_states._v;          // y direction
    suslr_st._ws = (v_params._cr * v_states._wx / 2.) + v_params._b * v_states._wy + v_states._w;  // z direction

    // RR
    susrr_st._us = (v_params._cr * v_states._wz / 2.) + v_states._u;  // x direction struct velocity in G-RF
    susrr_st._vs = -v_params._b * v_states._wz + v_states._v;         // y direction
    susrr_st._ws = (-v_params._cr * v_states._wx / 2.) + v_params._b * v_states._wy + v_states._w;  // z direction

    // Evaluate the length of the struct and its compression velocity in current time step
    // This is to get the unsprung mass velocity and accelerations in G-RF

    // Instantaneous length of the strut
    suslf_st._ls = suslf_st._lsi - (suslf_st._xs - suslf_st._xsi);
    susrf_st._ls = susrf_st._lsi - (susrf_st._xs - susrf_st._xsi);
    suslr_st._ls = suslr_st._lsi - (suslr_st._xs - suslr_st._xsi);
    susrr_st._ls = susrr_st._lsi - (susrr_st._xs - susrr_st._xsi);

    // Unsprung mass velocities in G-RF
    // x direction
    suslf_st._uu = suslf_st._us - suslf_st._ls * v_states._wy;  // LF
    susrf_st._uu = susrf_st._us - susrf_st._ls * v_states._wy;  // RF
    suslr_st._uu = suslr_st._us - suslr_st._ls * v_states._wy;  // LR
    susrr_st._uu = susrr_st._us - susrr_st._ls * v_states._wy;  // RR

    // y direction
    suslf_st._vu = suslf_st._vs + suslf_st._ls * v_states._wx;  // LF
    susrf_st._vu = susrf_st._vs + susrf_st._ls * v_states._wx;  // RF
    suslr_st._vu = suslr_st._vs + suslr_st._ls * v_states._wx;  // LR
    susrr_st._vu = susrr_st._vs + susrr_st._ls * v_states._wx;  // RR

    // z direction is a differential equation that is solved in suspension advance

    // Strut accelerations from vehicle accelerations
    // LF
    suslf_st._dus = (-v_params._cf * v_states._wzdot) / 2 + v_states._udot;
    suslf_st._dvs = v_params._a * v_states._wzdot + v_states._vdot;
    // RF
    susrf_st._dus = (v_params._cf * v_states._wzdot) / 2 + v_states._udot;
    susrf_st._dvs = v_params._a * v_states._wzdot + v_states._vdot;
    // LR
    suslr_st._dus = (-v_params._cr * v_states._wzdot) / 2 + v_states._udot;
    suslr_st._dvs = -v_params._b * v_states._wzdot + v_states._vdot;
    // RR
    susrr_st._dus = (v_params._cr * v_states._wzdot) / 2 + v_states._udot;
    susrr_st._dvs = (-v_params._b * v_states._wzdot) + v_states._vdot;

    // Unsprung mass acceleration in G-RF
    // RF
    susrf_st._duu = susrf_st._dus - (-susrf_st._dxs * v_states._wy + susrf_st._ls * v_states._wydot);
    susrf_st._dvu = susrf_st._dvs + (-susrf_st._dxs * v_states._wx + susrf_st._ls * v_states._wxdot);
    // LF
    suslf_st._duu = suslf_st._dus - (-suslf_st._dxs * v_states._wy + suslf_st._ls * v_states._wydot);
    suslf_st._dvu = suslf_st._dvs + (-suslf_st._dxs * v_states._wx + suslf_st._ls * v_states._wxdot);
    // LR
    suslr_st._duu = suslr_st._dus - (-suslr_st._dxs * v_states._wy + suslr_st._ls * v_states._wydot);
    suslr_st._dvu = suslr_st._dvs + (-suslr_st._dxs * v_states._wx + suslr_st._ls * v_states._wxdot);
    // RR
    susrr_st._duu = susrr_st._dus - (-susrr_st._dxs * v_states._wy + susrr_st._ls * v_states._wydot);
    susrr_st._dvu = susrr_st._dvs + (-susrr_st._dxs * v_states._wx + susrr_st._ls * v_states._wxdot);
}
void d24::vehToTireTransform(const VehicleState& v_states,
                             TMeasyState& tirelf_st,
                             TMeasyState& tirerf_st,
                             TMeasyState& tirelr_st,
                             TMeasyState& tirerr_st,
                             const SuspensionState& suslf_st,
                             const SuspensionState& susrf_st,
                             const SuspensionState& suslr_st,
                             const SuspensionState& susrr_st,
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

    // Velocities in tire frame from the vehicle and suspension
    // Derived from bond graph
    // https://www.tandfonline.com/doi/epdf/10.1080/00423110600882449?needAccess=true&role=button

    // LF
    tirelf_st._vgx =
        std::cos(v_states._theta) * (suslf_st._uu - v_states._wy * tirelf_st._rStat) +
        std::sin(v_states._theta) * (suslf_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirelf_st._rStat + suslf_st._vu));
    tirelf_st._vgy = std::cos(v_states._phi) * (suslf_st._vu + v_states._wx * tirelf_st._rStat) -
                     suslf_st._wu * std::sin(v_states._phi);
    // RF
    tirerf_st._vgx =
        std::cos(v_states._theta) * (susrf_st._uu - v_states._wy * tirerf_st._rStat) +
        std::sin(v_states._theta) * (susrf_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirerf_st._rStat + susrf_st._vu));
    tirerf_st._vgy = std::cos(v_states._phi) * (susrf_st._vu + v_states._wx * tirerf_st._rStat) -
                     susrf_st._wu * std::sin(v_states._phi);
    // LR
    tirelr_st._vgx =
        std::cos(v_states._theta) * (suslr_st._uu - v_states._wy * tirelr_st._rStat) +
        std::sin(v_states._theta) * (suslr_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirelr_st._rStat + suslr_st._vu));
    tirelr_st._vgy = std::cos(v_states._phi) * (suslr_st._vu + v_states._wx * tirelr_st._rStat) -
                     suslr_st._wu * std::sin(v_states._phi);
    // RR
    tirerr_st._vgx =
        std::cos(v_states._theta) * (susrr_st._uu - v_states._wy * tirerr_st._rStat) +
        std::sin(v_states._theta) * (susrr_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirerr_st._rStat + susrr_st._vu));
    tirerr_st._vgy = std::cos(v_states._phi) * (susrr_st._vu + v_states._wx * tirerr_st._rStat) -
                     susrr_st._wu * std::sin(v_states._phi);

    // Now we transform this to velocity in the tire contact patch

    // LF
    tirelf_st._vsy = tirelf_st._vgy;
    tirelf_st._vsx = tirelf_st._vgx * std::cos(delta) + tirelf_st._vgy * std::sin(delta);

    // RF
    tirerf_st._vsy = tirerf_st._vgy;
    tirerf_st._vsx = tirerf_st._vgx * std::cos(delta) + tirerf_st._vgy * std::sin(delta);

    // LR - No steer
    tirelr_st._vsy = tirelr_st._vgy;
    tirelr_st._vsx = tirelr_st._vgx;

    // RR - No steer
    tirerr_st._vsy = tirerr_st._vgy;
    tirerr_st._vsx = tirerr_st._vgx;
}

void d24::vehToTireTransform(const VehicleState& v_states,
                             TMeasyNrState& tirelf_st,
                             TMeasyNrState& tirerf_st,
                             TMeasyNrState& tirelr_st,
                             TMeasyNrState& tirerr_st,
                             const SuspensionState& suslf_st,
                             const SuspensionState& susrf_st,
                             const SuspensionState& suslr_st,
                             const SuspensionState& susrr_st,
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

    // Velocities in tire frame from the vehicle and suspension
    // Derived from bond graph
    // https://www.tandfonline.com/doi/epdf/10.1080/00423110600882449?needAccess=true&role=button

    // LF
    tirelf_st._vgx =
        std::cos(v_states._theta) * (suslf_st._uu - v_states._wy * tirelf_st._rStat) +
        std::sin(v_states._theta) * (suslf_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirelf_st._rStat + suslf_st._vu));
    tirelf_st._vgy = std::cos(v_states._phi) * (suslf_st._vu + v_states._wx * tirelf_st._rStat) -
                     suslf_st._wu * std::sin(v_states._phi);
    // RF
    tirerf_st._vgx =
        std::cos(v_states._theta) * (susrf_st._uu - v_states._wy * tirerf_st._rStat) +
        std::sin(v_states._theta) * (susrf_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirerf_st._rStat + susrf_st._vu));
    tirerf_st._vgy = std::cos(v_states._phi) * (susrf_st._vu + v_states._wx * tirerf_st._rStat) -
                     susrf_st._wu * std::sin(v_states._phi);
    // LR
    tirelr_st._vgx =
        std::cos(v_states._theta) * (suslr_st._uu - v_states._wy * tirelr_st._rStat) +
        std::sin(v_states._theta) * (suslr_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirelr_st._rStat + suslr_st._vu));
    tirelr_st._vgy = std::cos(v_states._phi) * (suslr_st._vu + v_states._wx * tirelr_st._rStat) -
                     suslr_st._wu * std::sin(v_states._phi);
    // RR
    tirerr_st._vgx =
        std::cos(v_states._theta) * (susrr_st._uu - v_states._wy * tirerr_st._rStat) +
        std::sin(v_states._theta) * (susrr_st._wu * std::cos(v_states._phi) +
                                     std::sin(v_states._phi) * (v_states._wx * tirerr_st._rStat + susrr_st._vu));
    tirerr_st._vgy = std::cos(v_states._phi) * (susrr_st._vu + v_states._wx * tirerr_st._rStat) -
                     susrr_st._wu * std::sin(v_states._phi);

    // Now we transform this to velocity in the tire contact patch

    // LF
    tirelf_st._vsy = tirelf_st._vgy;
    tirelf_st._vsx = tirelf_st._vgx * std::cos(delta) + tirelf_st._vgy * std::sin(delta);

    // RF
    tirerf_st._vsy = tirerf_st._vgy;
    tirerf_st._vsx = tirerf_st._vgx * std::cos(delta) + tirerf_st._vgy * std::sin(delta);

    // LR - No steer
    tirelr_st._vsy = tirelr_st._vgy;
    tirelr_st._vsx = tirelr_st._vgx;

    // RR - No steer
    tirerr_st._vsy = tirerr_st._vgy;
    tirerr_st._vsx = tirerr_st._vgx;
}

void d24::tireToVehTransform(const VehicleState& v_states,
                             TMeasyState& tirelf_st,
                             TMeasyState& tirerf_st,
                             TMeasyState& tirelr_st,
                             TMeasyState& tirerr_st,
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
    // These forces are still in the tire contact patch
    // left front
    _fx = tirelf_st._fx * std::cos(delta) - tirelf_st._fy * std::sin(delta);
    _fy = tirelf_st._fx * std::sin(delta) + tirelf_st._fy * std::cos(delta);
    tirelf_st._fx = _fx;
    tirelf_st._fy = _fy;

    // right front
    _fx = tirerf_st._fx * std::cos(delta) - tirerf_st._fy * std::sin(delta);
    _fy = tirerf_st._fx * std::sin(delta) + tirerf_st._fy * std::cos(delta);
    tirerf_st._fx = _fx;
    tirerf_st._fy = _fy;

    // rear tires - No steer so no need to transform

    // Now we need to get the tire ground forces in the vehicle inertial frame (G-RF)
    // LF
    tirelf_st._fxg = tirelf_st._fx * std::cos(v_states._theta) - tirelf_st._fz * std::sin(v_states._theta);
    tirelf_st._fyg = tirelf_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirelf_st._fy * std::cos(v_states._phi) +
                     tirelf_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirelf_st._fzg =
        tirelf_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirelf_st._fy * std::sin(v_states._phi) +
        tirelf_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere

    // RF
    tirerf_st._fxg = tirerf_st._fx * std::cos(v_states._theta) - tirerf_st._fz * std::sin(v_states._theta);
    tirerf_st._fyg = tirerf_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirerf_st._fy * std::cos(v_states._phi) +
                     tirerf_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirerf_st._fzg =
        tirerf_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirerf_st._fy * std::sin(v_states._phi) +
        tirerf_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere

    // LR
    tirelr_st._fxg = tirelr_st._fx * std::cos(v_states._theta) - tirelr_st._fz * std::sin(v_states._theta);
    tirelr_st._fyg = tirelr_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirelr_st._fy * std::cos(v_states._phi) +
                     tirelr_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirelr_st._fzg =
        tirelr_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirelr_st._fy * std::sin(v_states._phi) +
        tirelr_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere

    // RR
    tirerr_st._fxg = tirerr_st._fx * std::cos(v_states._theta) - tirerr_st._fz * std::sin(v_states._theta);
    tirerr_st._fyg = tirerr_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirerr_st._fy * std::cos(v_states._phi) +
                     tirerr_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirerr_st._fzg =
        tirerr_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirerr_st._fy * std::sin(v_states._phi) +
        tirerr_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere
}

void d24::tireToVehTransform(const VehicleState& v_states,
                             TMeasyNrState& tirelf_st,
                             TMeasyNrState& tirerf_st,
                             TMeasyNrState& tirelr_st,
                             TMeasyNrState& tirerr_st,
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
    // These forces are still in the tire contact patch
    // left front
    _fx = tirelf_st._fx * std::cos(delta) - tirelf_st._fy * std::sin(delta);
    _fy = tirelf_st._fx * std::sin(delta) + tirelf_st._fy * std::cos(delta);
    tirelf_st._fx = _fx;
    tirelf_st._fy = _fy;

    // right front
    _fx = tirerf_st._fx * std::cos(delta) - tirerf_st._fy * std::sin(delta);
    _fy = tirerf_st._fx * std::sin(delta) + tirerf_st._fy * std::cos(delta);
    tirerf_st._fx = _fx;
    tirerf_st._fy = _fy;

    // rear tires - No steer so no need to transform

    // Now we need to get the tire ground forces in the vehicle inertial frame (G-RF)
    // LF
    tirelf_st._fxg = tirelf_st._fx * std::cos(v_states._theta) - tirelf_st._fz * std::sin(v_states._theta);
    tirelf_st._fyg = tirelf_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirelf_st._fy * std::cos(v_states._phi) +
                     tirelf_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirelf_st._fzg =
        tirelf_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirelf_st._fy * std::sin(v_states._phi) +
        tirelf_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere

    // RF
    tirerf_st._fxg = tirerf_st._fx * std::cos(v_states._theta) - tirerf_st._fz * std::sin(v_states._theta);
    tirerf_st._fyg = tirerf_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirerf_st._fy * std::cos(v_states._phi) +
                     tirerf_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirerf_st._fzg =
        tirerf_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirerf_st._fy * std::sin(v_states._phi) +
        tirerf_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere

    // LR
    tirelr_st._fxg = tirelr_st._fx * std::cos(v_states._theta) - tirelr_st._fz * std::sin(v_states._theta);
    tirelr_st._fyg = tirelr_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirelr_st._fy * std::cos(v_states._phi) +
                     tirelr_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirelr_st._fzg =
        tirelr_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirelr_st._fy * std::sin(v_states._phi) +
        tirelr_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere

    // RR
    tirerr_st._fxg = tirerr_st._fx * std::cos(v_states._theta) - tirerr_st._fz * std::sin(v_states._theta);
    tirerr_st._fyg = tirerr_st._fx * std::sin(v_states._phi) * std::sin(v_states._theta) +
                     tirerr_st._fy * std::cos(v_states._phi) +
                     tirerr_st._fz * std::sin(v_states._phi) * std::cos(v_states._theta);
    tirerr_st._fzg =
        tirerr_st._fx * std::sin(v_states._theta) * std::cos(v_states._phi) - tirerr_st._fy * std::sin(v_states._phi) +
        tirerr_st._fz * std::cos(v_states._phi) * std::cos(v_states._theta);  // I dont think this is used anywhere
}

void d24::computeForcesThroughSus(const VehicleState& v_states,
                                  const TMeasyState& tirelf_st,
                                  const TMeasyState& tirerf_st,
                                  const TMeasyState& tirelr_st,
                                  const TMeasyState& tirerr_st,
                                  SuspensionState& suslf_st,
                                  SuspensionState& susrf_st,
                                  SuspensionState& suslr_st,
                                  SuspensionState& susrr_st,
                                  const VehicleParam& v_params,
                                  const TMeasyParam& t_params,
                                  const SuspensionParam& sus_params) {
    // Forces to be transformed to the chassis through the suspension struct points
    // LF
    suslf_st._fxs = tirelf_st._fxg + v_params._muf * G * std::sin(v_states._theta) - v_params._muf * suslf_st._duu +
                    v_params._muf * v_states._wz * suslf_st._vu - v_params._muf * v_states._wy * suslf_st._wu;
    suslf_st._fys = tirelf_st._fyg - v_params._muf * G * std::sin(v_states._phi) * std::cos(v_states._theta) -
                    v_params._muf * suslf_st._dvu + v_params._muf * v_states._wx * suslf_st._wu -
                    v_params._muf * v_states._wz * suslf_st._uu;
    suslf_st._fzs = suslf_st._xs * sus_params._ks + suslf_st._dxs * sus_params._bs;

    // RF
    susrf_st._fxs = tirerf_st._fxg + v_params._muf * G * std::sin(v_states._theta) - v_params._muf * susrf_st._duu +
                    v_params._muf * v_states._wz * susrf_st._vu - v_params._muf * v_states._wy * susrf_st._wu;
    susrf_st._fys = tirerf_st._fyg - v_params._muf * G * std::sin(v_states._phi) * std::cos(v_states._theta) -
                    v_params._muf * susrf_st._dvu + v_params._muf * v_states._wx * susrf_st._wu -
                    v_params._muf * v_states._wz * susrf_st._uu;
    susrf_st._fzs = susrf_st._xs * sus_params._ks + susrf_st._dxs * sus_params._bs;

    // LR
    suslr_st._fxs = tirelr_st._fxg + v_params._mur * G * std::sin(v_states._theta) - v_params._mur * suslr_st._duu +
                    v_params._mur * v_states._wz * suslr_st._vu - v_params._mur * v_states._wy * suslr_st._wu;
    suslr_st._fys = tirelr_st._fyg - v_params._mur * G * std::sin(v_states._phi) * std::cos(v_states._theta) -
                    v_params._mur * suslr_st._dvu + v_params._mur * v_states._wx * suslr_st._wu -
                    v_params._mur * v_states._wz * suslr_st._uu;
    suslr_st._fzs = suslr_st._xs * sus_params._ks + suslr_st._dxs * sus_params._bs;

    // RR
    susrr_st._fxs = tirerr_st._fxg + v_params._mur * G * std::sin(v_states._theta) - v_params._mur * susrr_st._duu +
                    v_params._mur * v_states._wz * susrr_st._vu - v_params._mur * v_states._wy * susrr_st._wu;
    susrr_st._fys = tirerr_st._fyg - v_params._mur * G * std::sin(v_states._phi) - v_params._mur * susrr_st._dvu +
                    v_params._mur * v_states._wx * susrr_st._wu - v_params._mur * v_states._wz * susrr_st._uu;
    susrr_st._fzs = susrr_st._xs * sus_params._ks + susrr_st._dxs * sus_params._bs;

    // Additional vertical load transfer that occurs at the wheels
    // These are jacking forces
    // https://www.tandfonline.com/doi/epdf/10.1080/00423110600882449?needAccess=true&role=button
    // RF
    susrf_st._fzd =
        (tirerf_st._fyg * tirerf_st._rStat + susrf_st._fys * susrf_st._ls + tirelf_st._fyg * tirelf_st._rStat +
         suslf_st._fys * suslf_st._ls - (susrf_st._fys + suslf_st._fys) * v_params._hrcf) /
        v_params._cf;
    // LF
    suslf_st._fzd = -susrf_st._fzd;

    // RR
    susrr_st._fzd =
        (tirerr_st._fyg * tirerr_st._rStat + susrr_st._fys * susrr_st._ls + tirelr_st._fyg * tirelr_st._rStat +
         suslr_st._fys * suslr_st._ls - (susrr_st._fys + suslr_st._fys) * v_params._hrcr) /
        v_params._cr;
    // LR
    suslr_st._fzd = -susrr_st._fzd;

    // Roll Moments transmitted to the sprung mass -> about x
    suslf_st._mx = suslf_st._fys * v_params._hrcf;
    susrf_st._mx = susrf_st._fys * v_params._hrcf;
    suslr_st._mx = suslr_st._fys * v_params._hrcr;
    susrr_st._mx = susrr_st._fys * v_params._hrcr;

    // y direction
    suslf_st._my = -(tirelf_st._fxg * tirelf_st._rStat + suslf_st._fxs * suslf_st._ls);
    susrf_st._my = -(tirerf_st._fxg * tirerf_st._rStat + susrf_st._fxs * susrf_st._ls);
    suslr_st._my = -(tirelr_st._fxg * tirelr_st._rStat + suslr_st._fxs * suslr_st._ls);
    susrr_st._my = -(tirerr_st._fxg * tirerr_st._rStat + susrr_st._fxs * susrr_st._ls);

    // z direction
    suslf_st._mz = 0;
    susrf_st._mz = 0;
    suslr_st._mz = 0;
    susrr_st._mz = 0;
}

void d24::computeForcesThroughSus(const VehicleState& v_states,
                                  const TMeasyNrState& tirelf_st,
                                  const TMeasyNrState& tirerf_st,
                                  const TMeasyNrState& tirelr_st,
                                  const TMeasyNrState& tirerr_st,
                                  SuspensionState& suslf_st,
                                  SuspensionState& susrf_st,
                                  SuspensionState& suslr_st,
                                  SuspensionState& susrr_st,
                                  const VehicleParam& v_params,
                                  const TMeasyNrParam& t_params,
                                  const SuspensionParam& sus_params) {
    // Forces to be transformed to the chassis through the suspension struct points
    // LF
    suslf_st._fxs = tirelf_st._fxg + v_params._muf * G * std::sin(v_states._theta) - v_params._muf * suslf_st._duu +
                    v_params._muf * v_states._wz * suslf_st._vu - v_params._muf * v_states._wy * suslf_st._wu;
    suslf_st._fys = tirelf_st._fyg - v_params._muf * G * std::sin(v_states._phi) * std::cos(v_states._theta) -
                    v_params._muf * suslf_st._dvu + v_params._muf * v_states._wx * suslf_st._wu -
                    v_params._muf * v_states._wz * suslf_st._uu;
    suslf_st._fzs = suslf_st._xs * sus_params._ks + suslf_st._dxs * sus_params._bs;

    // RF
    susrf_st._fxs = tirerf_st._fxg + v_params._muf * G * std::sin(v_states._theta) - v_params._muf * susrf_st._duu +
                    v_params._muf * v_states._wz * susrf_st._vu - v_params._muf * v_states._wy * susrf_st._wu;
    susrf_st._fys = tirerf_st._fyg - v_params._muf * G * std::sin(v_states._phi) * std::cos(v_states._theta) -
                    v_params._muf * susrf_st._dvu + v_params._muf * v_states._wx * susrf_st._wu -
                    v_params._muf * v_states._wz * susrf_st._uu;
    susrf_st._fzs = susrf_st._xs * sus_params._ks + susrf_st._dxs * sus_params._bs;

    // LR
    suslr_st._fxs = tirelr_st._fxg + v_params._mur * G * std::sin(v_states._theta) - v_params._mur * suslr_st._duu +
                    v_params._mur * v_states._wz * suslr_st._vu - v_params._mur * v_states._wy * suslr_st._wu;
    suslr_st._fys = tirelr_st._fyg - v_params._mur * G * std::sin(v_states._phi) * std::cos(v_states._theta) -
                    v_params._mur * suslr_st._dvu + v_params._mur * v_states._wx * suslr_st._wu -
                    v_params._mur * v_states._wz * suslr_st._uu;
    suslr_st._fzs = suslr_st._xs * sus_params._ks + suslr_st._dxs * sus_params._bs;

    // RR
    susrr_st._fxs = tirerr_st._fxg + v_params._mur * G * std::sin(v_states._theta) - v_params._mur * susrr_st._duu +
                    v_params._mur * v_states._wz * susrr_st._vu - v_params._mur * v_states._wy * susrr_st._wu;
    susrr_st._fys = tirerr_st._fyg - v_params._mur * G * std::sin(v_states._phi) - v_params._mur * susrr_st._dvu +
                    v_params._mur * v_states._wx * susrr_st._wu - v_params._mur * v_states._wz * susrr_st._uu;
    susrr_st._fzs = susrr_st._xs * sus_params._ks + susrr_st._dxs * sus_params._bs;

    // Additional vertical load transfer that occurs at the wheels
    // These are jacking forces
    // https://www.tandfonline.com/doi/epdf/10.1080/00423110600882449?needAccess=true&role=button
    // RF
    susrf_st._fzd =
        (tirerf_st._fyg * tirerf_st._rStat + susrf_st._fys * susrf_st._ls + tirelf_st._fyg * tirelf_st._rStat +
         suslf_st._fys * suslf_st._ls - (susrf_st._fys + suslf_st._fys) * v_params._hrcf) /
        v_params._cf;
    // LF
    suslf_st._fzd = -susrf_st._fzd;

    // RR
    susrr_st._fzd =
        (tirerr_st._fyg * tirerr_st._rStat + susrr_st._fys * susrr_st._ls + tirelr_st._fyg * tirelr_st._rStat +
         suslr_st._fys * suslr_st._ls - (susrr_st._fys + suslr_st._fys) * v_params._hrcr) /
        v_params._cr;
    // LR
    suslr_st._fzd = -susrr_st._fzd;

    // Roll Moments transmitted to the sprung mass -> about x
    suslf_st._mx = suslf_st._fys * v_params._hrcf;
    susrf_st._mx = susrf_st._fys * v_params._hrcf;
    suslr_st._mx = suslr_st._fys * v_params._hrcr;
    susrr_st._mx = susrr_st._fys * v_params._hrcr;

    // y direction
    suslf_st._my = -(tirelf_st._fxg * tirelf_st._rStat + suslf_st._fxs * suslf_st._ls);
    susrf_st._my = -(tirerf_st._fxg * tirerf_st._rStat + susrf_st._fxs * susrf_st._ls);
    suslr_st._my = -(tirelr_st._fxg * tirelr_st._rStat + suslr_st._fxs * suslr_st._ls);
    susrr_st._my = -(tirerr_st._fxg * tirerr_st._rStat + susrr_st._fxs * susrr_st._ls);

    // z direction
    suslf_st._mz = 0;
    susrf_st._mz = 0;
    suslr_st._mz = 0;
    susrr_st._mz = 0;
}
// -----------------------------------------------------------------------------
// Tire RHS functions and its helper functions
// -----------------------------------------------------------------------------

void d24::tmxy_combined(double& f, double& fos, double s, double df0, double sm, double fm, double ss, double fs) {
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

void d24::computeCombinedCoulombForce(double& fx,
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

void d24::computeTireRHS(const VehicleState& v_states,
                         TMeasyState& t_states,
                         const VehicleParam& v_params,
                         const TMeasyParam& t_params,
                         double steering) {
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    // Get the transformed variables out
    t_states._fz = t_states._xt * t_params._kt;  // Tire vertical force computed here
    double fz = t_states._fz;                    // vertical force
    double vsy = t_states._vsy;                  // y slip velocity
    double vsx = t_states._vsx;                  // x slip velocity
    t_states._rStat = (t_params._r0 - t_states._xt) / (std::cos(v_states._theta) * std::cos(v_states._phi));

    // If the wheel is off contact, set all states to 0 and return
    if (fz <= 0) {
        t_states._xe = 0;
        t_states._ye = 0;
        t_states._omega = 0;
        t_states._xt = 0;
        t_states._rStat = t_params._r0;
        t_states._fz = 0;
        t_states._fx = 0;
        t_states._fy = 0;
        t_states._My = 0;
        return;
    }

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
    // integrated xe, unline in the semi-implicit solver
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

void d24::computeTireRHS(const VehicleState& v_states,
                         TMeasyNrState& t_states,
                         const VehicleParam& v_params,
                         const TMeasyNrParam& t_params,
                         double steering) {
    double delta = 0;
    if (v_params._nonLinearSteer) {
        // Extract steer map
        std::vector<MapEntry> steer_map = v_params._steerMap;
        delta = getMapY(steer_map, steering);
    } else {
        delta = steering * v_params._maxSteer;
    }

    // Get the transformed variables out
    t_states._fz = t_states._xt * t_params._kt;  // Tire vertical force computed here
    double fz = t_states._fz;                    // vertical force
    double vsy = t_states._vsy;                  // y slip velocity
    double vsx = t_states._vsx;                  // x slip velocity
    t_states._rStat = (t_params._r0 - t_states._xt) / (std::cos(v_states._theta) * std::cos(v_states._phi));

    // If the wheel is off contact, set all states to 0 and return
    if (fz <= 0) {
        t_states._omega = 0;
        t_states._xt = 0;
        t_states._rStat = t_params._r0;
        t_states._fx = 0;
        t_states._fy = 0;
        t_states._fz = 0;
        t_states._My = 0;
        return;
    }

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
    computeCombinedCoulombForce(Fx0, Fy0, t_params._mu, vsx, vsy, fz, t_params._vcoulomb);

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

    // Compute the coefficient to "blend" the coulomb force with the slip force
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

    // Now that we have both the coulomb force and the slip force, we can combine them with the sine blend to prevent
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
void d24::computeTireCompressionVelocity(const VehicleState& v_states,
                                         TMeasyState& tirelf_st,
                                         TMeasyState& tirerf_st,
                                         TMeasyState& tirelr_st,
                                         TMeasyState& tirerr_st,
                                         const SuspensionState& suslf_st,
                                         const SuspensionState& susrf_st,
                                         const SuspensionState& suslr_st,
                                         const SuspensionState& susrr_st) {
    tirelf_st._dxt = tirelf_st._vsz -
                     (std::cos(v_states._theta) * (suslf_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + suslf_st._vu * std::sin(v_states._phi)) -
                      suslf_st._uu * std::sin(v_states._theta));

    tirerf_st._dxt = tirerf_st._vsz -
                     (std::cos(v_states._theta) * (susrf_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + susrf_st._vu * std::sin(v_states._phi)) -
                      susrf_st._uu * std::sin(v_states._theta));

    tirelr_st._dxt = tirelr_st._vsz -
                     (std::cos(v_states._theta) * (suslr_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + suslr_st._vu * std::sin(v_states._phi)) -
                      suslr_st._uu * std::sin(v_states._theta));

    tirerr_st._dxt = tirerr_st._vsz -
                     (std::cos(v_states._theta) * (susrr_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + susrr_st._vu * std::sin(v_states._phi)) -
                      susrr_st._uu * std::sin(v_states._theta));
}

void d24::computeTireCompressionVelocity(const VehicleState& v_states,
                                         TMeasyNrState& tirelf_st,
                                         TMeasyNrState& tirerf_st,
                                         TMeasyNrState& tirelr_st,
                                         TMeasyNrState& tirerr_st,
                                         const SuspensionState& suslf_st,
                                         const SuspensionState& susrf_st,
                                         const SuspensionState& suslr_st,
                                         const SuspensionState& susrr_st) {
    tirelf_st._dxt = tirelf_st._vsz -
                     (std::cos(v_states._theta) * (suslf_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + suslf_st._vu * std::sin(v_states._phi)) -
                      suslf_st._uu * std::sin(v_states._theta));

    tirerf_st._dxt = tirerf_st._vsz -
                     (std::cos(v_states._theta) * (susrf_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + susrf_st._vu * std::sin(v_states._phi)) -
                      susrf_st._uu * std::sin(v_states._theta));

    tirelr_st._dxt = tirelr_st._vsz -
                     (std::cos(v_states._theta) * (suslr_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + suslr_st._vu * std::sin(v_states._phi)) -
                      suslr_st._uu * std::sin(v_states._theta));

    tirerr_st._dxt = tirerr_st._vsz -
                     (std::cos(v_states._theta) * (susrr_st._wu * std::cos(v_states._phi)  // vsz is 0 for a smooth road
                                                   + susrr_st._vu * std::sin(v_states._phi)) -
                      susrr_st._uu * std::sin(v_states._theta));
}

void d24::computeSusRHS(const VehicleState& v_states,
                        const TMeasyState& tirelf_st,
                        const TMeasyState& tirerf_st,
                        const TMeasyState& tirelr_st,
                        const TMeasyState& tirerr_st,
                        SuspensionState& suslf_st,
                        SuspensionState& susrf_st,
                        SuspensionState& suslr_st,
                        SuspensionState& susrr_st,
                        const VehicleParam& v_params,
                        const SuspensionParam& sus_params) {
    // Lets extract the cardan angles as they are a pain to type
    double theta = v_states._theta;
    double phi = v_states._phi;
    double psi = v_states._psi;

    // Unsprung mass vertical velocity differential equation
    suslf_st._dwu =
        (1 / v_params._muf) *
        (std::cos(phi) * (std::cos(theta) * (tirelf_st._fz - v_params._muf * G) + std::sin(theta) * tirelf_st._fx) -
         std::sin(phi) * tirelf_st._fy - suslf_st._fzd - suslf_st._xs * sus_params._ks -
         suslf_st._dxs * sus_params._bs - v_params._muf * (suslf_st._vu * v_states._wx - suslf_st._uu * v_states._wy));

    susrf_st._dwu =
        (1 / v_params._muf) *
        (std::cos(phi) * (std::cos(theta) * (tirerf_st._fz - v_params._muf * G) + std::sin(theta) * tirerf_st._fx) -
         std::sin(phi) * tirerf_st._fy - susrf_st._fzd - susrf_st._xs * sus_params._ks -
         susrf_st._dxs * sus_params._bs - v_params._muf * (susrf_st._vu * v_states._wx - susrf_st._uu * v_states._wy));

    suslr_st._dwu =
        (1 / v_params._mur) *
        (std::cos(phi) * (std::cos(theta) * (tirelr_st._fz - v_params._mur * G) + std::sin(theta) * tirelr_st._fx) -
         std::sin(phi) * tirelr_st._fy - suslr_st._fzd - suslr_st._xs * sus_params._ks -
         suslr_st._dxs * sus_params._bs - v_params._mur * (suslr_st._vu * v_states._wx - suslr_st._uu * v_states._wy));

    susrr_st._dwu =
        (1 / v_params._mur) *
        (std::cos(phi) * (std::cos(theta) * (tirerr_st._fz - v_params._mur * G) + std::sin(theta) * tirerr_st._fx) -
         std::sin(phi) * tirerr_st._fy - susrr_st._fzd - susrr_st._xs * sus_params._ks -
         susrr_st._dxs * sus_params._bs - v_params._mur * (susrr_st._vu * v_states._wx - susrr_st._uu * v_states._wy));

    // instantaneous suspension spring deflection
    suslf_st._dxs = -suslf_st._ws + suslf_st._wu;
    susrf_st._dxs = -susrf_st._ws + susrf_st._wu;
    suslr_st._dxs = -suslr_st._ws + suslr_st._wu;
    susrr_st._dxs = -susrr_st._ws + susrr_st._wu;
}

void d24::computeSusRHS(const VehicleState& v_states,
                        const TMeasyNrState& tirelf_st,
                        const TMeasyNrState& tirerf_st,
                        const TMeasyNrState& tirelr_st,
                        const TMeasyNrState& tirerr_st,
                        SuspensionState& suslf_st,
                        SuspensionState& susrf_st,
                        SuspensionState& suslr_st,
                        SuspensionState& susrr_st,
                        const VehicleParam& v_params,
                        const SuspensionParam& sus_params) {
    // Lets extract the cardan angles as they are a pain to type
    double theta = v_states._theta;
    double phi = v_states._phi;
    double psi = v_states._psi;

    // Unsprung mass vertical velocity differential equation
    suslf_st._dwu =
        (1 / v_params._muf) *
        (std::cos(phi) * (std::cos(theta) * (tirelf_st._fz - v_params._muf * G) + std::sin(theta) * tirelf_st._fx) -
         std::sin(phi) * tirelf_st._fy - suslf_st._fzd - suslf_st._xs * sus_params._ks -
         suslf_st._dxs * sus_params._bs - v_params._muf * (suslf_st._vu * v_states._wx - suslf_st._uu * v_states._wy));

    susrf_st._dwu =
        (1 / v_params._muf) *
        (std::cos(phi) * (std::cos(theta) * (tirerf_st._fz - v_params._muf * G) + std::sin(theta) * tirerf_st._fx) -
         std::sin(phi) * tirerf_st._fy - susrf_st._fzd - susrf_st._xs * sus_params._ks -
         susrf_st._dxs * sus_params._bs - v_params._muf * (susrf_st._vu * v_states._wx - susrf_st._uu * v_states._wy));

    suslr_st._dwu =
        (1 / v_params._mur) *
        (std::cos(phi) * (std::cos(theta) * (tirelr_st._fz - v_params._mur * G) + std::sin(theta) * tirelr_st._fx) -
         std::sin(phi) * tirelr_st._fy - suslr_st._fzd - suslr_st._xs * sus_params._ks -
         suslr_st._dxs * sus_params._bs - v_params._mur * (suslr_st._vu * v_states._wx - suslr_st._uu * v_states._wy));

    susrr_st._dwu =
        (1 / v_params._mur) *
        (std::cos(phi) * (std::cos(theta) * (tirerr_st._fz - v_params._mur * G) + std::sin(theta) * tirerr_st._fx) -
         std::sin(phi) * tirerr_st._fy - susrr_st._fzd - susrr_st._xs * sus_params._ks -
         susrr_st._dxs * sus_params._bs - v_params._mur * (susrr_st._vu * v_states._wx - susrr_st._uu * v_states._wy));

    // instantaneous suspension spring deflection
    suslf_st._dxs = -suslf_st._ws + suslf_st._wu;
    susrf_st._dxs = -susrf_st._ws + susrf_st._wu;
    suslr_st._dxs = -suslr_st._ws + suslr_st._wu;
    susrr_st._dxs = -susrr_st._ws + susrr_st._wu;
}

// -----------------------------------------------------------------------------
// Powertrain and helper functions
// -----------------------------------------------------------------------------
// Returns drive torque at a given omega
double d24::driveTorque(const VehicleParam& v_params, const double throttle, const double motor_speed) {
    double motor_torque = 0.;
    // If we have throttle modulation like in a motor
    if (v_params._throttleMod) {
        std::vector<MapEntry> powertrain_map = v_params._powertrainMap;

        size_t len = v_params._powertrainMap.size();
        for (size_t i = 0; i < len; i++) {
            powertrain_map[i]._x = v_params._powertrainMap[i]._x * throttle;
            powertrain_map[i]._y = v_params._powertrainMap[i]._y * throttle;
        }

        // interpolate in the torque map to get the torque at this particular
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

// Function that calculates the torque split to each tire based on the
// differential max bias Exactly the same as Chrono implementation
void d24::differentialSplit(double torque,
                            double max_bias,
                            double speed_left,
                            double speed_right,
                            double& torque_left,
                            double& torque_right) {
    double diff = std::abs(speed_left - speed_right);

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

    if (std::abs(speed_left) < std::abs(speed_right)) {
        torque_left = slow;
        torque_right = fast;
    } else {
        torque_left = fast;
        torque_right = slow;
    }
}
// Computes the powertrain and produces the wheel angular velocity differential equations
void d24::computePowertrainRHS(VehicleState& v_states,
                               TMeasyState& tirelf_st,
                               TMeasyState& tirerf_st,
                               TMeasyState& tirelr_st,
                               TMeasyState& tirerr_st,
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
        double omega_t = 0.25 * (tirelf_st._omega + tirerf_st._omega + tirelr_st._omega + tirerr_st._omega);

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
        v_states._crankOmega = 0.25 * (tirelf_st._omega + tirerf_st._omega + tirelr_st._omega + tirerr_st._omega) /
                               v_params._gearRatios[v_states._current_gr];

        // The torque after transmission will then just become as there is no torque
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

    // torque split between the  front and rear (always half)
    double torque_front = 0;
    double torque_rear = 0;
    if (v_params._driveType) {  // If we have a 4WD vehicle split torque equally
        torque_front = torque_t * 0.5;
        torque_rear = torque_t * 0.5;
    } else {
        if (v_params._whichWheels) {
            torque_front = 0;
            torque_rear = torque_t;
        } else {
            torque_front = torque_t;
            torque_rear = 0;
        }
    }

    // first the front wheels
    differentialSplit(torque_front, max_bias, tirelf_st._omega, tirerf_st._omega, tirelf_st._engTor, tirerf_st._engTor);
    // then rear wheels
    differentialSplit(torque_rear, max_bias, tirelr_st._omega, tirerr_st._omega, tirelr_st._engTor, tirerr_st._engTor);

    // now use this force for our omegas
    // Get dOmega for each tire
    tirelf_st._dOmega = (1 / t_params._jw) * (tirelf_st._engTor + tirelf_st._My -
                                              sgn(tirelf_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirelf_st._fx * tirelf_st._rStat);

    tirerf_st._dOmega = (1 / t_params._jw) * (tirerf_st._engTor + tirerf_st._My -
                                              sgn(tirerf_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirerf_st._fx * tirerf_st._rStat);

    tirelr_st._dOmega = (1 / t_params._jw) * (tirelr_st._engTor + tirelr_st._My -
                                              sgn(tirelr_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirelr_st._fx * tirelr_st._rStat);

    tirerr_st._dOmega = (1 / t_params._jw) * (tirerr_st._engTor + tirerr_st._My -
                                              sgn(tirerr_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirerr_st._fx * tirerr_st._rStat);
}

void d24::computePowertrainRHS(VehicleState& v_states,
                               TMeasyNrState& tirelf_st,
                               TMeasyNrState& tirerf_st,
                               TMeasyNrState& tirelr_st,
                               TMeasyNrState& tirerr_st,
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
        double omega_t = 0.25 * (tirelf_st._omega + tirerf_st._omega + tirelr_st._omega + tirerr_st._omega);

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
        v_states._crankOmega = 0.25 * (tirelf_st._omega + tirerf_st._omega + tirelr_st._omega + tirerr_st._omega) /
                               v_params._gearRatios[v_states._current_gr];

        // The torque after transmission will then just become as there is no torque
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

    // torque split between the  front and rear (always half)
    double torque_front = 0;
    double torque_rear = 0;
    if (v_params._driveType) {  // If we have a 4WD vehicle split torque equally
        torque_front = torque_t * 0.5;
        torque_rear = torque_t * 0.5;
    } else {
        if (v_params._whichWheels) {
            torque_front = 0;
            torque_rear = torque_t;
        } else {
            torque_front = torque_t;
            torque_rear = 0;
        }
    }

    // first the front wheels
    differentialSplit(torque_front, max_bias, tirelf_st._omega, tirerf_st._omega, tirelf_st._engTor, tirerf_st._engTor);
    // then rear wheels
    differentialSplit(torque_rear, max_bias, tirelr_st._omega, tirerr_st._omega, tirelr_st._engTor, tirerr_st._engTor);

    // now use this force for our omegas
    // Get dOmega for each tire
    tirelf_st._dOmega = (1 / t_params._jw) * (tirelf_st._engTor + tirelf_st._My -
                                              sgn(tirelf_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirelf_st._fx * tirelf_st._rStat);

    tirerf_st._dOmega = (1 / t_params._jw) * (tirerf_st._engTor + tirerf_st._My -
                                              sgn(tirerf_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirerf_st._fx * tirerf_st._rStat);

    tirelr_st._dOmega = (1 / t_params._jw) * (tirelr_st._engTor + tirelr_st._My -
                                              sgn(tirelr_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirelr_st._fx * tirelr_st._rStat);

    tirerr_st._dOmega = (1 / t_params._jw) * (tirerr_st._engTor + tirerr_st._My -
                                              sgn(tirerr_st._omega) * brakeTorque(v_params, controls.m_braking) -
                                              tirerr_st._fx * tirerr_st._rStat);
}

void d24::computeVehicleRHS(VehicleState& v_states,
                            const TMeasyState& tirelf_st,
                            const TMeasyState& tirerf_st,
                            const TMeasyState& tirelr_st,
                            const TMeasyState& tirerr_st,
                            const SuspensionState& suslf_st,
                            const SuspensionState& susrf_st,
                            const SuspensionState& suslr_st,
                            const SuspensionState& susrr_st,
                            const VehicleParam& v_params,
                            const TMeasyParam& t_params,
                            const SuspensionParam& sus_params) {
    // Chassis dynamics equations
    v_states._udot = v_states._wz * v_states._v - v_states._wy * v_states._w +
                     (1 / v_params._m) * (suslf_st._fxs + susrf_st._fxs + suslr_st._fxs + susrr_st._fxs) +
                     G * std::sin(v_states._theta);

    v_states._vdot = v_states._wx * v_states._w - v_states._wz * v_states._u +
                     (1 / v_params._m) * (suslf_st._fys + susrf_st._fys + suslr_st._fys + susrr_st._fys) -
                     G * std::sin(v_states._phi) * std::cos(v_states._theta);

    v_states._wdot = v_states._wy * v_states._u - v_states._wx * v_states._v +
                     (1 / v_params._m) * (suslf_st._fzs + susrf_st._fzs + suslr_st._fzs + susrr_st._fzs +
                                          suslf_st._fzd + susrf_st._fzd + suslr_st._fzd + susrr_st._fzd) -
                     G * std::cos(v_states._phi) * std::cos(v_states._theta);

    v_states._wxdot = (1 / v_params._jx) * ((suslf_st._mx + susrf_st._mx + suslr_st._mx + susrr_st._mx) +
                                            (suslf_st._fzs - susrf_st._fzs) * (v_params._cf / 2.) +
                                            (suslr_st._fzs - susrr_st._fzs) * (v_params._cr / 2.));

    v_states._wydot = (1 / v_params._jy) *
                      ((suslf_st._my + susrf_st._my + suslr_st._my + susrr_st._my) +
                       (suslr_st._fzs + susrr_st._fzs) * v_params._b - (suslf_st._fzs + susrf_st._fzs) * v_params._a);

    v_states._wzdot =
        (1 / v_params._jz) *
        ((suslf_st._mz + susrf_st._mz + suslr_st._mz + susrr_st._mz) + (suslf_st._fys + susrf_st._fys) * v_params._a -
         (suslr_st._fys + susrr_st._fys) * v_params._b + (-suslf_st._fxs + susrf_st._fxs) * (v_params._cf / 2.) +
         (-suslr_st._fxs + susrr_st._fxs) * (v_params._cr / 2.));

    v_states._dtheta = v_states._wy * std::cos(v_states._phi) - v_states._wz * std::sin(v_states._phi);
    v_states._dpsi = (v_states._wy * std::sin(v_states._phi) / std::cos(v_states._theta)) +
                     (v_states._wz * std::cos(v_states._phi) / std::cos(v_states._theta));  // Trouble when theta is 90?
    v_states._dphi = v_states._wx + v_states._wy * std::sin(v_states._phi) * std::tan(v_states._theta) +
                     v_states._wz * std::cos(v_states._phi) * std::tan(v_states._theta);

    // Get the global X and Y -> I do not know how to get the global Z
    v_states._dx = (v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi));

    v_states._dy = (v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi));
}

void d24::computeVehicleRHS(VehicleState& v_states,
                            const TMeasyNrState& tirelf_st,
                            const TMeasyNrState& tirerf_st,
                            const TMeasyNrState& tirelr_st,
                            const TMeasyNrState& tirerr_st,
                            const SuspensionState& suslf_st,
                            const SuspensionState& susrf_st,
                            const SuspensionState& suslr_st,
                            const SuspensionState& susrr_st,
                            const VehicleParam& v_params,
                            const TMeasyNrParam& t_params,
                            const SuspensionParam& sus_params) {
    // Chassis dynamics equations
    v_states._udot = v_states._wz * v_states._v - v_states._wy * v_states._w +
                     (1 / v_params._m) * (suslf_st._fxs + susrf_st._fxs + suslr_st._fxs + susrr_st._fxs) +
                     G * std::sin(v_states._theta);

    v_states._vdot = v_states._wx * v_states._w - v_states._wz * v_states._u +
                     (1 / v_params._m) * (suslf_st._fys + susrf_st._fys + suslr_st._fys + susrr_st._fys) -
                     G * std::sin(v_states._phi) * std::cos(v_states._theta);

    v_states._wdot = v_states._wy * v_states._u - v_states._wx * v_states._v +
                     (1 / v_params._m) * (suslf_st._fzs + susrf_st._fzs + suslr_st._fzs + susrr_st._fzs +
                                          suslf_st._fzd + susrf_st._fzd + suslr_st._fzd + susrr_st._fzd) -
                     G * std::cos(v_states._phi) * std::cos(v_states._theta);

    v_states._wxdot = (1 / v_params._jx) * ((suslf_st._mx + susrf_st._mx + suslr_st._mx + susrr_st._mx) +
                                            (suslf_st._fzs - susrf_st._fzs) * (v_params._cf / 2.) +
                                            (suslr_st._fzs - susrr_st._fzs) * (v_params._cr / 2.));

    v_states._wydot = (1 / v_params._jy) *
                      ((suslf_st._my + susrf_st._my + suslr_st._my + susrr_st._my) +
                       (suslr_st._fzs + susrr_st._fzs) * v_params._b - (suslf_st._fzs + susrf_st._fzs) * v_params._a);

    v_states._wzdot =
        (1 / v_params._jz) *
        ((suslf_st._mz + susrf_st._mz + suslr_st._mz + susrr_st._mz) + (suslf_st._fys + susrf_st._fys) * v_params._a -
         (suslr_st._fys + susrr_st._fys) * v_params._b + (-suslf_st._fxs + susrf_st._fxs) * (v_params._cf / 2.) +
         (-suslr_st._fxs + susrr_st._fxs) * (v_params._cr / 2.));

    v_states._dtheta = v_states._wy * std::cos(v_states._phi) - v_states._wz * std::sin(v_states._phi);
    v_states._dpsi = (v_states._wy * std::sin(v_states._phi) / std::cos(v_states._theta)) +
                     (v_states._wz * std::cos(v_states._phi) / std::cos(v_states._theta));  // Trouble when theta is 90?
    v_states._dphi = v_states._wx + v_states._wy * std::sin(v_states._phi) * std::tan(v_states._theta) +
                     v_states._wz * std::cos(v_states._phi) * std::tan(v_states._theta);

    // Get the global X and Y -> I do not know how to get the global Z
    v_states._dx = (v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi));

    v_states._dy = (v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi));
}

// -----------------------------------------------------------------------------
// JSON file parsing
// -----------------------------------------------------------------------------

// ----------
// Tire
// ----------
// setting Tire parameters using a JSON file
void d24::setTireParamsJSON(TMeasyParam& t_params, const char* fileName) {
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

    
    t_params._jw = d["jw"].GetDouble();
    t_params._rr = d["rr"].GetDouble();
    t_params._r0 = d["r0"].GetDouble();
    t_params._pn = d["pn"].GetDouble();
    t_params._pnmax = d["pnmax"].GetDouble();
    t_params._cx = d["cx"].GetDouble();
    t_params._cy = d["cy"].GetDouble();
    t_params._kt = d["kt"].GetDouble();
    t_params._dx = d["dx"].GetDouble();
    t_params._dy = d["dy"].GetDouble();
    t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
    t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
    t_params._fzRdynco = d["fzRdynco"].GetDouble();
    t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble();

    t_params._dfx0Pn = d["dfx0Pn"].GetDouble();
    t_params._dfx0P2n = d["dfx0P2n"].GetDouble();
    t_params._fxmPn = d["fxmPn"].GetDouble();
    t_params._fxmP2n = d["fxmP2n"].GetDouble();
    t_params._fxsPn = d["fxsPn"].GetDouble();
    t_params._fxsP2n = d["fxsP2n"].GetDouble();
    t_params._sxmPn = d["sxmPn"].GetDouble();
    t_params._sxmP2n = d["sxmP2n"].GetDouble();
    t_params._sxsPn = d["sxsPn"].GetDouble();
    t_params._sxsP2n = d["sxsP2n"].GetDouble();

    t_params._dfy0Pn = d["dfy0Pn"].GetDouble();
    t_params._dfy0P2n = d["dfy0P2n"].GetDouble();
    t_params._fymPn = d["fymPn"].GetDouble();
    t_params._fymP2n = d["fymP2n"].GetDouble();
    t_params._fysPn = d["fysPn"].GetDouble();
    t_params._fysP2n = d["fysP2n"].GetDouble();
    t_params._symPn = d["symPn"].GetDouble();
    t_params._symP2n = d["symP2n"].GetDouble();
    t_params._sysPn = d["sysPn"].GetDouble();
    t_params._sysP2n = d["sysP2n"].GetDouble();
}

// setting Tire parameters using a JSON file for a TMeasyNr
void d24::setTireParamsJSON(TMeasyNrParam& t_params, const char* fileName) {
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
            t_params._jw = d["jw"].GetDouble();
            t_params._rr = d["rr"].GetDouble();
            t_params._mu = d["mu"].GetDouble();
            t_params._r0 = d["r0"].GetDouble();
            t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
            t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
            t_params._fzRdynco = d["fzRdynco"].GetDouble();
            t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble();

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
            t_params._jw = d["jw"].GetDouble();
            t_params._rr = d["rr"].GetDouble();
            t_params._r0 = d["r0"].GetDouble();
            t_params._pn = d["pn"].GetDouble();
            t_params._pnmax = d["pnmax"].GetDouble();
            t_params._kt = d["kt"].GetDouble();
            t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
            t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
            t_params._fzRdynco = d["fzRdynco"].GetDouble();
            t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble();

            t_params._dfx0Pn = d["dfx0Pn"].GetDouble();
            t_params._dfx0P2n = d["dfx0P2n"].GetDouble();
            t_params._fxmPn = d["fxmPn"].GetDouble();
            t_params._fxmP2n = d["fxmP2n"].GetDouble();
            t_params._fxsPn = d["fxsPn"].GetDouble();
            t_params._fxsP2n = d["fxsP2n"].GetDouble();
            t_params._sxmPn = d["sxmPn"].GetDouble();
            t_params._sxmP2n = d["sxmP2n"].GetDouble();
            t_params._sxsPn = d["sxsPn"].GetDouble();
            t_params._sxsP2n = d["sxsP2n"].GetDouble();

            t_params._dfy0Pn = d["dfy0Pn"].GetDouble();
            t_params._dfy0P2n = d["dfy0P2n"].GetDouble();
            t_params._fymPn = d["fymPn"].GetDouble();
            t_params._fymP2n = d["fymP2n"].GetDouble();
            t_params._fysPn = d["fysPn"].GetDouble();
            t_params._fysP2n = d["fysP2n"].GetDouble();
            t_params._symPn = d["symPn"].GetDouble();
            t_params._symP2n = d["symP2n"].GetDouble();
            t_params._sysPn = d["sysPn"].GetDouble();
            t_params._sysP2n = d["sysP2n"].GetDouble();
        }

    } else {
        t_params._jw = d["jw"].GetDouble();
        t_params._rr = d["rr"].GetDouble();
        t_params._r0 = d["r0"].GetDouble();
        t_params._pn = d["pn"].GetDouble();
        t_params._pnmax = d["pnmax"].GetDouble();
        t_params._kt = d["kt"].GetDouble();
        t_params._rdyncoPn = d["rdyncoPn"].GetDouble();
        t_params._rdyncoP2n = d["rdyncoP2n"].GetDouble();
        t_params._fzRdynco = d["fzRdynco"].GetDouble();
        t_params._rdyncoCrit = d["rdyncoCrit"].GetDouble();

        t_params._dfx0Pn = d["dfx0Pn"].GetDouble();
        t_params._dfx0P2n = d["dfx0P2n"].GetDouble();
        t_params._fxmPn = d["fxmPn"].GetDouble();
        t_params._fxmP2n = d["fxmP2n"].GetDouble();
        t_params._fxsPn = d["fxsPn"].GetDouble();
        t_params._fxsP2n = d["fxsP2n"].GetDouble();
        t_params._sxmPn = d["sxmPn"].GetDouble();
        t_params._sxmP2n = d["sxmP2n"].GetDouble();
        t_params._sxsPn = d["sxsPn"].GetDouble();
        t_params._sxsP2n = d["sxsP2n"].GetDouble();

        t_params._dfy0Pn = d["dfy0Pn"].GetDouble();
        t_params._dfy0P2n = d["dfy0P2n"].GetDouble();
        t_params._fymPn = d["fymPn"].GetDouble();
        t_params._fymP2n = d["fymP2n"].GetDouble();
        t_params._fysPn = d["fysPn"].GetDouble();
        t_params._fysP2n = d["fysP2n"].GetDouble();
        t_params._symPn = d["symPn"].GetDouble();
        t_params._symP2n = d["symP2n"].GetDouble();
        t_params._sysPn = d["sysPn"].GetDouble();
        t_params._sysP2n = d["sysP2n"].GetDouble();
    }
}

// Utility functions that guess the tire parameters for a TMeasy tire based on standard tire specifications that user
// can get from a spec sheet These functions are directly copy pasted from Chrono with minor modifications

// Function to compute the max tire load from the load index specified by the user
double d24::GetTireMaxLoad(unsigned int li) {
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
void d24::GuessTruck80Par(unsigned int li,          // tire load index
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
void d24::GuessTruck80Par(double tireLoad,   // tire load index
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
    t_params._dfx0Pn = 17.7764 * t_params._pn;
    t_params._dfx0P2n = 14.5301 * 2.0 * t_params._pn;
    t_params._fxmPn = 0.89965 * t_params._pn;
    t_params._fxmP2n = 0.77751 * 2.0 * t_params._pn;
    t_params._fxsPn = 0.46183 * t_params._pn;
    t_params._fxsP2n = 0.42349 * 2.0 * t_params._pn;
    t_params._sxmPn = 0.10811;
    t_params._sxmP2n = 0.12389;
    t_params._sxsPn = 0.66667;
    t_params._sxsP2n = 0.66667;
    t_params._dfy0Pn = 7.4013 * t_params._pn;
    t_params._dfy0P2n = 6.8505 * 2.0 * t_params._pn;
    t_params._fymPn = 0.75876 * t_params._pn;
    t_params._fymP2n = 0.72628 * 2.0 * t_params._pn;
    t_params._fysPn = 0.68276 * t_params._pn;
    t_params._fysP2n = 0.65319 * 2.0 * t_params._pn;
    t_params._symPn = 0.33167;
    t_params._symP2n = 0.33216;
    t_params._sysPn = 1.0296;
    t_params._sysP2n = 1.0296;
}

// Guessing tire parameters for a passenger car
void d24::GuessPassCar70Par(unsigned int li,          // tire load index
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
void d24::GuessPassCar70Par(double tireLoad,            // tire load index
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
    t_params._dfx0Pn = 18.3741 * t_params._pn;
    t_params._dfx0P2n = 19.4669 * 2.0 * t_params._pn;
    t_params._fxmPn = 1.1292 * t_params._pn;
    t_params._fxmP2n = 1.0896 * 2.0 * t_params._pn;
    t_params._fxsPn = 0.80149 * t_params._pn;
    t_params._fxsP2n = 0.76917 * 2.0 * t_params._pn;
    t_params._sxmPn = 0.13913;
    t_params._sxmP2n = 0.13913;
    t_params._sxsPn = 0.66667;
    t_params._sxsP2n = 0.66667;
    t_params._dfy0Pn = 15.9826 * t_params._pn;
    t_params._dfy0P2n = 12.8509 * 2.0 * t_params._pn;
    t_params._fymPn = 1.0009 * t_params._pn;
    t_params._fymP2n = 0.91367 * 2.0 * t_params._pn;
    t_params._fysPn = 0.8336 * t_params._pn;
    t_params._fysP2n = 0.77336 * 2.0 * t_params._pn;
    t_params._symPn = 0.14852;
    t_params._symP2n = 0.18504;
    t_params._sysPn = 0.96524;
    t_params._sysP2n = 1.0714;
}
// ----------
// Suspension
// ----------
// setting Tire parameters using a JSON file
void d24::setSuspensionParamsJSON(SuspensionParam& sus_params, const char* fileName) {
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

    
    sus_params._ks = d["ks"].GetDouble();
    sus_params._bs = d["bs"].GetDouble();
}

// ----------
// Vehicle
// ----------
// setting Vehicle parameters using a JSON file
void d24::setVehParamsJSON(VehicleParam& v_params, const char* fileName) {
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
    v_params._jx = d["jx"].GetDouble();
    v_params._jy = d["jy"].GetDouble();
    v_params._cf = d["cf"].GetDouble();
    v_params._cr = d["cr"].GetDouble();
    v_params._muf = d["muf"].GetDouble();
    v_params._mur = d["mur"].GetDouble();
    v_params._hrcf = d["hrcf"].GetDouble();
    v_params._hrcr = d["hrcr"].GetDouble();

    // Non linear steering which maps the normalized steering input to wheel angle
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
    // If there is no non linear steer then the normalized steering input is just multiplied by the max steering wheel
    // angle
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
        // If this is the case then just set the shift map for all gears to the same value
        for (unsigned int i = 0; i < gears; i++) {
            MapEntry m;
            m._x = d["downshiftRPM"].GetDouble() * rpm2rad;
            m._y = d["upshiftRPM"].GetDouble() * rpm2rad;
            v_params._shiftMap.push_back(m);
        }
    } else {
        // If there is no upshiftRPM and downshiftRPM then there should be a shift map
        // and this should have as many entries as the number of gears
        for (unsigned int i = 0; i < gears; i++) {
            MapEntry m;
            m._x = d["shiftMap"][i][0u].GetDouble() * rpm2rad;  // downshift
            m._y = d["shiftMap"][i][1u].GetDouble() * rpm2rad;  // upshift
            v_params._shiftMap.push_back(m);
        }
    }

    v_params._tcbool = d["tcBool"].GetBool();

    v_params._maxBrakeTorque = d["maxBrakeTorque"].GetDouble();

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