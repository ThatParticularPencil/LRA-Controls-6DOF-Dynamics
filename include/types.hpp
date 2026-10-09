#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dynamics{

struct Environment { // ! unused if input forces contains gravity
    Eigen::Vector3d g_inertial{0.0, 0.0, -9.80665};        // ENU. For NED use {0, 0, +9.80665}
};

struct Heft{
    double mass, CoR;       //CoR is along the Z axis //!TODO do we need this?
    Eigen::Matrix3d I;      //inertia about the CoR
    Eigen::Matrix3d inv_I;  //inverse of inertia matrix
};

struct State{
    using Vec3 = Eigen::Vector3d;

    /*
    pos is inertial frame
    Xe, Ye, Ze

    vel is inertial frame
    U, V, W

    omega is inertial frame
    x, y, z

    q rotates body vectors into inertial.
    w, x, y, z
    */
    Vec3 pos, vel, omega;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Heft heft;
    double time;
};

struct Inputs{
    using Vec3 = Eigen::Vector3d;

    Vec3 Force, Moment;
    double mass_dot;      //!TODO
    double CoR_dot;       //
};

} 