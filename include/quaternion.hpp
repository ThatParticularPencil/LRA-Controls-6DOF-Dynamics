#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dynamics {

// scalar-first in constructors (w, x, y, z).
// q converts body vectors to inertial:  vec_inertial = q * vec_body.

// q_dot = 0.5 * q (x) [0, w_body]
Eigen::Quaterniond q_deriv(const Eigen::Quaterniond& q,
                           const Eigen::Vector3d& omega_body);

// where s is the scale factor
Eigen::Quaterniond q_add_scaled(const Eigen::Quaterniond& q,
                                const Eigen::Quaterniond& dq,
                                double s);

Eigen::Vector3d body_to_inertial(const Eigen::Quaterniond& q,
                                 const Eigen::Vector3d& v_body);

Eigen::Vector3d inertial_to_body(const Eigen::Quaterniond& q,
                                 const Eigen::Vector3d& v_inertial);

// 3-2-1 (yaw-pitch-roll) angles, radians.
Eigen::Vector3d q_to_euler321(const Eigen::Quaterniond& q);

}
