#pragma once
#include "types.hpp"
#include "eom.hpp"

#include <cmath>
#include <stdexcept>

namespace dynamics {

struct StepConfig {
    double max_substep = 0.01;   // [seconds] largest internal RK4 step
};

// Advances x by dt [s] under inputs u.
//
// Frames and units (SI throughout):
//   x.pos, x.vel   inertial frame (ENU, z up)
//   x.omega        body frame [rad/s]
//   x.q            body -> inertial, Hamilton, unit length
//   u.Force [N], u.Moment [N*m]   body frame, moments about the CG
//   u.mass_dot [kg/s], u.CoR_dot [m/s]   rates
//
// If dt > cfg.max_substep, dt is split into equal substeps. u is held
// constant across all of them. heft.I is not integrated.
//
// Preconditions: dt > 0, x.heft.mass > 0, all inputs finite.


State step(const State& x, const Inputs& u, const Environment& env,
           double dt, const StepConfig& cfg = {});

//! precon check /!may be unnecessary
void check_step_preconditions(const State& x, const Inputs& u,
                              const Environment& env,
                              double dt, const StepConfig& cfg); 

} 
