#include "quaternion.hpp"

#include <algorithm>
#include <cmath>

namespace dynamics {

Eigen::Quaterniond q_deriv(const Eigen::Quaterniond& q,
                           const Eigen::Vector3d& omega_body) {
    const Eigen::Quaterniond omega(0.0, omega_body.x(), omega_body.y(), omega_body.z());
    Eigen::Quaterniond qdot = q * omega;
    qdot.coeffs() *= 0.5;
    return qdot;
}

Eigen::Quaterniond q_add_scaled(const Eigen::Quaterniond& q,
                                const Eigen::Quaterniond& dq,
                                double s) {
    Eigen::Quaterniond r;
    r.coeffs() = q.coeffs() + s * dq.coeffs();
    return r;
}

Eigen::Vector3d body_to_inertial(const Eigen::Quaterniond& q,
                                 const Eigen::Vector3d& v_body) {
    return q * v_body;
}

Eigen::Vector3d inertial_to_body(const Eigen::Quaterniond& q,
                                 const Eigen::Vector3d& v_inertial) {
    return q.conjugate() * v_inertial;
}

Eigen::Vector3d q_to_euler321(const Eigen::Quaterniond& q) {
    const double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    const double roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double s     = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch = std::asin(s);
    const double yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return {roll, pitch, yaw};
}

}
