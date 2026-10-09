#pragma once
#include "types.hpp"
#include "eom.hpp"

namespace dynamics {

// Returns x + s * xdot, component-wise
State add_scaled(const State& x, const StateDot& xdot, double s);

// The returned quaternion is normalized.
State rk4_step(const State& x, const Inputs& u,
               const Environment& env, double dt);

}