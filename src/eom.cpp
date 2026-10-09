#include "eom.hpp"
#include "quaternion.hpp"

namespace dynamics {

StateDot f(const State& x, const Inputs& u, const Environment& env) {
    StateDot d;

    // Translation (inertial)

    // Attitude

    // Rotation (body) should be in quaternion.cpp

    // rates of doubles
    return d;
}

}
