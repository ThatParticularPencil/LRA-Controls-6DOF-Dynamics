#include "integrator.hpp"
#include "quaternion.hpp"

namespace dynamics {
namespace {

StateDot rk4_sum(const StateDot& k1, const StateDot& k2,
                 const StateDot& k3, const StateDot& k4) {
    StateDot s;
    s.pos_dot = k1.pos_dot + 2.0 * k2.pos_dot + 2.0 * k3.pos_dot + k4.pos_dot;
    s.vel_dot = k1.vel_dot + 2.0 * k2.vel_dot + 2.0 * k3.vel_dot + k4.vel_dot;
    s.omega_dot = k1.omega_dot + 2.0 * k2.omega_dot + 2.0 * k3.omega_dot + k4.omega_dot;
    s.q_dot.coeffs() = k1.q_dot.coeffs() + 2.0 * k2.q_dot.coeffs()
                     + 2.0 * k3.q_dot.coeffs() + k4.q_dot.coeffs();
    s.time_dot = k1.time_dot + 2.0 * k2.time_dot + 2.0 * k3.time_dot + k4.time_dot;
    s.mass_dot = k1.mass_dot + 2.0 * k2.mass_dot + 2.0 * k3.mass_dot + k4.mass_dot;
    s.CoR_dot = k1.CoR_dot + 2.0 * k2.CoR_dot + 2.0 * k3.CoR_dot + k4.CoR_dot;
    return s;
}

} // namespace

State add_scaled(const State& x, const StateDot& xdot, double s) {
    State y = x;
    y.pos = x.pos + s * xdot.pos_dot;
    y.vel = x.vel + s * xdot.vel_dot;
    y.omega = x.omega + s * xdot.omega_dot;
    y.q = q_add_scaled(x.q, xdot.q_dot, s);
    y.heft.mass = x.heft.mass + s * xdot.mass_dot;
    y.heft.CoR = x.heft.CoR + s * xdot.CoR_dot;
    y.time = x.time + s * xdot.time_dot;
    return y;
}

State rk4_step(const State& x, const Inputs& u,
               const Environment& env, double dt) {
    const StateDot k1 = f(x, u, env);
    const StateDot k2 = f(add_scaled(x, k1, 0.5 * dt), u, env);
    const StateDot k3 = f(add_scaled(x, k2, 0.5 * dt), u, env);
    const StateDot k4 = f(add_scaled(x, k3, dt), u, env);
    State y = add_scaled(x, rk4_sum(k1, k2, k3, k4), dt / 6.0);
    y.q.normalize();
    return y;
}

} // namespace dynamics
