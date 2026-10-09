#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "types.hpp"

namespace dynamics {

struct StateDot {
    Eigen::Vector3d    pos_dot, vel_dot, omega_dot;
    Eigen::Quaterniond q_dot;
    double             time_dot, mass_dot, CoR_dot;
};

// Continuous-time equations of motion.
// u.Force [N] and u.Moment [N*m] are body frame; Moment about the CoR.
// u.mass_dot [kg/s] and u.CoR_dot [m/s] are rates.
// heft.I is held constant across one step.
StateDot f(const State& x, const Inputs& u, const Environment& env);

}
