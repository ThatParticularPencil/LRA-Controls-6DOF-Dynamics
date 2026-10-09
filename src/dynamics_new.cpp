#include "dynamics.hpp"
#include "integrator.hpp"

#include <cmath>

namespace dynamics {

void check_step_preconditions(const State& x, const Inputs& u,      //!TODO check; this may not be necessary
                              const Environment& env,
                              double dt, const StepConfig& cfg) {
    if (!(dt > 0.0)) {
        throw std::invalid_argument("step precondition failed: dt < 0");
    }
    if (!(x.heft.mass > 0.0)) {
        throw std::invalid_argument("step precondition failed: mass < 0");
    }

    const bool finite =
        std::isfinite(dt) && std::isfinite(cfg.max_substep) &&
        std::isfinite(x.time) && std::isfinite(x.heft.mass) && std::isfinite(x.heft.CoR) &&
        std::isfinite(u.mass_dot) && std::isfinite(u.CoR_dot) &&
        x.pos.allFinite() && x.vel.allFinite() && x.omega.allFinite() &&
        x.q.coeffs().allFinite() &&
        x.heft.I.allFinite() && x.heft.inv_I.allFinite() &&
        u.Force.allFinite() && u.Moment.allFinite() &&
        env.g_inertial.allFinite();
    if (!finite) {
        throw std::invalid_argument("step precondition failed: inputs non-finite");
    }
}

//main function that computes state update
State step(const State& x, const Inputs& u, const Environment& env,
           double dt, const StepConfig& cfg) {
    check_step_preconditions(x, u, env, dt, cfg);

    /*sub-divide time step into n "max_substep" steps
    this is for rk4 accuracy. big step means low acc*/
    int n = 1;
    if (cfg.max_substep > 0.0 && dt > cfg.max_substep) {
        n = static_cast<int>(std::ceil(dt / cfg.max_substep));
    }
    const double h = dt / static_cast<double>(n);

    State state_new = x;
    for (int i = 0; i < n; ++i) {
        state_new = rk4_step(state_new, u, env, h);
    }
    return state_new;
}

} 