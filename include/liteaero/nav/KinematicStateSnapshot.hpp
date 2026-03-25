#pragma once

#include <liteaero/nav/GeodeticPosition.hpp>
#include <Eigen/Geometry>

namespace liteaero::nav {

/// Minimal non-redundant kinematic state — data only.
///
/// All values are in SI units. Orientation is stored as quaternions to avoid
/// Euler-angle singularities. q_nb is not stored because it is exactly
/// derivable from q_nw, alpha_rad, and beta_rad — see KinematicStateUtil::q_nb().
///
/// Derived quantities (Euler angles, frame-transformed velocities, plane of
/// motion, turn circle) are computed on demand by free functions in namespace
/// liteaero::nav::KinematicStateUtil.
///
/// Total size: 112 bytes.
struct KinematicStateSnapshot {

    // ── Time ────────────────────────────────────────────────────────────────
    double time_s = 0.0;                ///< Simulation time, seconds

    // ── Position ────────────────────────────────────────────────────────────
    GeodeticPosition position;          ///< WGS84 geodetic position (lat/lon/alt)

    // ── Translational state ─────────────────────────────────────────────────
    Eigen::Vector3f velocity_ned_mps       = Eigen::Vector3f::Zero();  ///< Ground velocity, NED, m/s
    Eigen::Vector3f acceleration_ned_mps2  = Eigen::Vector3f::Zero();  ///< Acceleration, NED, m/s²

    // ── Orientation ─────────────────────────────────────────────────────────
    /// Wind-to-NED rotation quaternion. q_nb is derived via KinematicStateUtil::q_nb().
    Eigen::Quaternionf q_nw = Eigen::Quaternionf::Identity();

    // ── Angular rates ───────────────────────────────────────────────────────
    Eigen::Vector3f rates_body_rps = Eigen::Vector3f::Zero();  ///< Body angular rates, rad/s

    // ── Aerodynamic angles and rates ────────────────────────────────────────
    float alpha_rad            = 0.0f;  ///< Angle of attack, radians
    float beta_rad             = 0.0f;  ///< Sideslip angle, radians
    float alpha_dot_rad_s      = 0.0f;  ///< Angle of attack rate, rad/s
    float beta_dot_rad_s       = 0.0f;  ///< Sideslip angle rate, rad/s
    float roll_rate_wind_rad_s = 0.0f;  ///< Roll rate of Wind frame w.r.t. NED, rad/s

    // ── Wind ────────────────────────────────────────────────────────────────
    Eigen::Vector3f wind_ned_mps = Eigen::Vector3f::Zero();  ///< Ambient wind, NED, m/s
};

} // namespace liteaero::nav
