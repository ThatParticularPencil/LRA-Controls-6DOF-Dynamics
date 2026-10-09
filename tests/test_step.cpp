#include "dynamics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

dynamics::State make_state() {
    dynamics::State state;
    state.pos.setZero();
    state.vel = dynamics::State::Vec3(10.0, 0.0, 0.0);
    state.omega.setZero();
    state.heft.mass = 1200.0;
    state.heft.CoR = 0.0;
    state.heft.I.setIdentity();
    state.heft.inv_I.setIdentity();
    state.time = 0.0;
    return state;
}

dynamics::Inputs make_input() {
    dynamics::Inputs input;
    input.Force = dynamics::Inputs::Vec3(5000.0, 0.0, 0.0);
    input.Moment.setZero();
    input.mass_dot = 0.0;
    input.CoR_dot = 0.0;
    return input;
}

bool throws_invalid(const dynamics::State& state, const dynamics::Inputs& input,
                    const dynamics::Environment& env, double dt) {
    try {
        dynamics::step(state, input, env, dt);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

} // namespace

int main() {
    const dynamics::State state = make_state();
    const dynamics::Inputs input = make_input();
    dynamics::Environment env;
    env.g_inertial.setZero();

    const dynamics::StateDot dot = dynamics::f(state, input, env);
    std::cout << "vel_dot = " << dot.vel_dot.transpose() << "\n";
    std::cout << "omega_dot = " << dot.omega_dot.transpose() << "\n";

    const double dt = 0.01;
    const double ax = input.Force.x() / state.heft.mass;
    const dynamics::State stepped = dynamics::step(state, input, env, dt);
    const double vx_expected = state.vel.x() + ax * dt;
    const double x_expected = state.pos.x() + state.vel.x() * dt + 0.5 * ax * dt * dt;

    std::cout << "x = " << stepped.pos.x() << "\n";
    std::cout << "vx = " << stepped.vel.x() << "\n";

    if (std::abs(stepped.pos.x() - x_expected) > 1e-9 ||
        std::abs(stepped.vel.x() - vx_expected) > 1e-9) {
        std::cerr << "translation step mismatch\n";
        return 1;
    }

    if (!throws_invalid(state, input, env, 0.0)) {
        std::cerr << "dt > 0 was not enforced\n";
        return 1;
    }

    dynamics::State light = state;
    light.heft.mass = 0.0;
    if (!throws_invalid(light, input, env, dt)) {
        std::cerr << "mass > 0 was not enforced\n";
        return 1;
    }

    dynamics::Inputs bad = input;
    bad.Force.x() = std::numeric_limits<double>::quiet_NaN();
    if (!throws_invalid(state, bad, env, dt)) {
        std::cerr << "finite inputs were not enforced\n";
        return 1;
    }

    return 0;
}
